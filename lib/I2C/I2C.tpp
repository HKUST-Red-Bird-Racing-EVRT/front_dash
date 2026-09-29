/**
 * @file I2C.tpp
 * @author Planeson (carson.cpk@proton.me)
 * @brief Implementation of the I2C class template and I2cTransaction struct.
 * @version 2.0.0.beta1
 * @date 2026-09-04
 *
 * @copyright Copyright (c) 2026 Red Bird Racing
 *
 */

#include "I2C.hpp"
#include <avr/io.h>      //redundant include (in .hpp), but keep for clarity
#include <util/atomic.h> // for ATOMIC_BLOCK

// Template parameter definitions, used to simplify the template syntax in the implementation file.
#define TEMPLATE_DEF uint16_t BITRATE_KBPS, uint8_t QUEUE_SIZE, uint8_t WATCHDOG_MAX_COUNT
#define TEMPLATES BITRATE_KBPS, QUEUE_SIZE, WATCHDOG_MAX_COUNT

/**
 * @brief Constexpr constructor for I2cTransaction.
 * @note The length parameter for reads is stored as the number of ACKs expected, which is one less than the total byte count for reads. This allows the ISR to determine when to send ACK vs NACK without needing additional state tracking. For writes, the length is stored directly as provided.
 *
 * @param[in] address_and_mode The 7-bit I2C address combined with the read/write mode in the least significant bit (0 for write, 1 for read).
 * @param[in] length The number of bytes to transfer.
 * @param[in,out] data Pointer to the data buffer. For writes, this is the source buffer. For reads, this is the destination buffer where received data will be stored. Be sure that the buffer is large enough to hold the specified length of data.
 */
constexpr I2cTransaction::I2cTransaction(const uint8_t address_and_mode_, const uint8_t length_, uint8_t *const data_)
    : address_and_mode(address_and_mode_),
      length(length_),
      data(data_)
{
}

/**
 * @brief Function to cause a compilation error if a read transaction is created with a length of zero.
 */
extern void __ERROR_I2C_READ_LENGTH_MUST_BE_GREATER_THAN_ZERO__()
    __attribute__((error("I2C read length must be greater than zero!")));

/**
 * @brief Creates a new I2cTransaction for a write operation.
 *
 * @param address Address of the I2C device to write to (7-bit address).
 * @param length Number of bytes to write. 0 <= length < 256.
 * @param source Pointer to the source data buffer.
 * @return The constructed I2cTransaction object for the write operation.
 */
inline constexpr I2cTransaction I2cTransaction::makeWrite(const uint8_t address, const uint8_t length, const uint8_t *const source)
{
    return I2cTransaction(address << 1, length, const_cast<uint8_t *>(source));
}

/**
 * @brief Creates a new I2cTransaction for a read operation.
 *
 * @param address Address of the I2C device to read from (7-bit address).
 * @param length Number of bytes to read. 0 < length < 256.
 * @param destination Pointer to the destination buffer where the read data will be stored.
 * @attention The destination buffer must be large enough to hold the specified number of bytes.
 * @note If you need to use a repeated start to read from a specific register, use the makeChainedWrite() function to write the register address first, followed by a read transaction.
 * @return The constructed I2cTransaction object for the read operation.
 */
inline constexpr I2cTransaction I2cTransaction::makeRead(const uint8_t address, const uint8_t length, uint8_t *const destination)
{
    if (__builtin_constant_p(length) && length == 0)
    {
        __ERROR_I2C_READ_LENGTH_MUST_BE_GREATER_THAN_ZERO__();
    }
    return I2cTransaction((address << 1) | 0x01, length - 1, destination);
}

/**
 * @brief Constexpr constructor for I2C. Initializes the TWI hardware with the specified bitrate and sets up internal state.
 * @note This must be called before pump.
 */
template <TEMPLATE_DEF>
constexpr I2C<TEMPLATES>::I2C()
{
    init();
}

/**
 * @brief Returns the number of empty slots in the queue. Used to check if there is enough space to push a group of transactions for an all-or-nothing approach to pushing transactions.
 *
 * @return the number of empty slots in the queue
 */
template <TEMPLATE_DEF>
constexpr uint8_t I2C<TEMPLATES>::queueEmptySlots() const
{
    uint8_t filled = (queue_write_index - queue_read_index) & QUEUE_MASK;
    return QUEUE_SIZE - 1 - filled;
}

/**
 * @brief Checks if the queue is empty.
 *
 * @return whether the queue is empty
 */
template <TEMPLATE_DEF>
inline constexpr bool I2C<TEMPLATES>::queueEmpty() const
{
    return queue_write_index == queue_read_index;
}

/**
 * @brief Pushes a new transaction to the queue.

 * @param[in] new_queuer new I2cTransaction to be added to the queue.
 * @note The queue is a ring buffer fifo. Users should check for available slots with emptySlots(), and employ an all-or-nothing approach to pushing groups of transactions.
 * @return true if the transaction was successfully added to the queue, false if the queue is full.
 */
template <TEMPLATE_DEF>
constexpr bool I2C<TEMPLATES>::push(const I2cTransaction &new_queuer)
{
    uint8_t new_write_index = (queue_write_index + 1) & QUEUE_MASK;
    if (new_write_index == queue_read_index)
    { // queue full
        return false;
    }
    // update the content first, then update the index, to ensure that the ISR sees a valid transaction when it reads the index
    queue[queue_write_index] = new_queuer;
    queue_write_index = new_write_index;
    return true;
}

/**
 * @brief Clears the transaction queue, effectively discarding any pending transactions. This can be used to prioritize urgent transactions by clearing the queue before pushing them.
 * For instance, if the I2C screen needs a layout change, the old data updates can be purged from the queue to prevent old data from overwriting the new layout.
 *
 * @note since the assignment is atomic, this function is safe to call from the main loop, as the current instruction will still be finished.
 *
 */
template <TEMPLATE_DEF>
constexpr void I2C<TEMPLATES>::clearQueue()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) // to prevent ISR updating read index while we are clearing the queue, which would cause a race condition
    {
        queue_write_index = queue_read_index;
    }
}

/**
 * @brief Provides a heartbeat for the I2C driver, checks for bus hangs, and initiates transactions from the queues if the bus is idle.
 * @attention This function must be called regularly in the main loop to ensure proper operation of the I2C driver, especially for watchdog functionality and to kickstart transactions when the bus is idle.
 */
template <TEMPLATE_DEF>
void I2C<TEMPLATES>::pump()
{
    switch (bus_state)
    {
    case I2cState::Idle:
    {
        if (!queueEmpty())
        { // used to not have any tasks, kickstart task
            watchdog_count = 0;
            bus_state = I2cState::Busy;
            setActiveJob(queue[queue_read_index]);
            // set control register last to prevent another interrupt from not updating active_job
            TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
            return;
        }
        return;
    }
    case I2cState::Busy:
    {
        if (watchdog_pulsed)
        {
            watchdog_pulsed = false;
            watchdog_count = 0;
            return;
        }

        if (watchdog_count > WATCHDOG_MAX_COUNT)
        {
            bus_state = I2cState::Hung;
            recovery_state = RecoveryState::Init;
            watchdog_count = 0;
            recoverBus();
        }
        else
        {
            ++watchdog_count;
        }
        return;
    }
    case I2cState::Hung:
    {
        recoverBus();
        return;
    }
    default:
        __builtin_unreachable();
    }
    return;
}

/**
 * @brief handles the TWI interrupt service routine.
 * @attention Must be called from within the ISR(TWI_vect) block. See example.
 * @note NACK / arbitration lost results in infinite repeated start - intended behaviour,
 * as the I2C devices are considered essential, disconnected I2C hanging the system is the ideal behaviour of this don't-care condition.
 */
template <TEMPLATE_DEF>
inline void I2C<TEMPLATES>::handleIsr()
{
    watchdog_pulsed = true;
    I2cStatus bus_status = static_cast<I2cStatus>(TWSR & 0xF8);
    switch (bus_status)
    {
    case I2cStatus::Start:
    case I2cStatus::RepeatedStart:
    { // transaction just started, address slave + mode
        active_byte_index = 0;
        TWDR = active_address_and_mode;
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
        return;
    }
    case I2cStatus::AddressWriteAck:
    case I2cStatus::DataSentAck:
    { //  slave acknowledged address or data, send next byte, checking if is the last to send
        if (active_byte_index < active_length)
        {
            TWDR = active_data_ptr[active_byte_index];
            ++active_byte_index;
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
        }
        else
        {
            finishIsr();
        }
        return;
    }
    case I2cStatus::AddressReadAck:
    { // slave acknowledged address for read, read data, checking if is the last to read
        if (active_length > 0)
        { // next ACK, multi-byte read
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
        }
        else
        { // next NACK, only read 1 byte
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
        }
        return;
    }
    case I2cStatus::DataReadAck:
    { // read data from slave, checking if is the last to read
        active_data_ptr[active_byte_index] = TWDR;
        ++active_byte_index;
        if (active_byte_index < active_length)
        { // next byte still need ACK
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
        }
        else
        { // next byte need NACK to end
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
        }
        return;
    }
    case I2cStatus::DataReadNack:
    { // read last byte from slave, no more bytes to read, finish transaction
        active_data_ptr[active_byte_index] = TWDR;
        finishIsr();
        return;
    }
    case I2cStatus::ArbitrationLost:
    case I2cStatus::AddressWriteNack:
    case I2cStatus::AddressReadNack:
    case I2cStatus::DataSentNack:
    { // slave did not acknowledge address or data, or arbitration lost, restart transaction
        TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
        return;
    }
    default:
    { // unexpected status, stop the bus and reset the state machine, this should never happen
        TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN) | (1 << TWIE);
        bus_state = I2cState::Idle;
    }
    }
}

/**
 * @brief Private helper to initialize the TWI hardware with the specified bitrate and sets up internal state.
 */
template <TEMPLATE_DEF>
constexpr void I2C<TEMPLATES>::init()
{
    static_assert(BITRATE_KBPS <= 400, "Max I2C bitrate of the ATmega328p is 400kbps!");
    // check if "undoing" the operation gives the correct speed, if not then it means the rate is invalid
    static_assert(((((F_CPU / BITRATE_KBPS / 1000) - 16) / 2) * 2 + 16) * 1000 * BITRATE_KBPS == F_CPU, "This bitrate is impossible to achieve for this CPU speed!");

    // bitrate
    TWBR = ((F_CPU / BITRATE_KBPS / 1000) - 16) / 2;

    // prescaler
    TWSR &= ~(1 << TWPS1) & ~(1 << TWPS0);

    // enable TWI, disabling GPIO functionalities
    TWCR = (1 << TWEN);
    bus_state = I2cState::Idle;
}

/**
 * @brief Private helper to finish the current I2C transaction, choosing the next transaction (if any), and update the internal state.
 */
template <TEMPLATE_DEF>
inline void I2C<TEMPLATES>::finishIsr()
{
    queue_read_index = (queue_read_index + 1) & QUEUE_MASK;
    if (!queueEmpty())
    {
        setActiveJob(queue[queue_read_index]);
        TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
        return;
    }

    // no more transactions, stop the bus and reset the state machine
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    bus_state = I2cState::Idle;
    return;
}

/**
 * @brief Private helper to recover the I2C bus in case of a hang or other error condition. This function is called when the watchdog detects that the bus is hung.
 * @note nop for now. Hardware seems to handle this well enough, but if it becomes an issue, implement a recovery state machine here to recover the bus; leaving this code in the current state helps will identifying and debugging the issue if it arises.
 */
template <TEMPLATE_DEF>
inline void I2C<TEMPLATES>::recoverBus()
{
    return;
    // nop function, see note
    // switch (recovery_state)
    // {
    // case RecoveryState::Init:
    // {

    //     break;
    // }
    // }
}

/**
 * @brief Private helper to set the active job for the I2C transaction. This function updates the internal state of the I2C driver to reflect the current transaction being processed.
 * @note From a design perspective, the active job struct is separated into its components to save on instructions used on read with offset, especially as the members are used in the ISR directly.
 * @param job The I2cTransaction to set as the active job.
 */
template <TEMPLATE_DEF>
inline void I2C<TEMPLATES>::setActiveJob(const I2cTransaction &job)
{
    active_address_and_mode = job.address_and_mode;
    active_length = job.length;
    active_data_ptr = job.data.read; // For writes, this will be cast to const uint8_t* in the ISR

    active_byte_index = 0;
}

#undef TEMPLATE_DEF
#undef TEMPLATES