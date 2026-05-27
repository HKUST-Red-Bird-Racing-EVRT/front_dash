/**
 * @file Page.hpp
 * @author chiho
 * @brief Unified page class for all display pages
 * @version 1.0
 * @date 2026-05-27
 * @see Page.tpp
 * 
 * - Page 0: Driver
 * - Page 1: VCU
 * - Page 2: BMS
 * - Page 3: empty
 * - Page 4: snake
 */

#ifndef PAGE_HPP
#define PAGE_HPP

#include <cstdint>

class CarState;
class LiquidCrystal_I2C;

/**
 * @brief BMS status enum
 */
enum class BmsStatus : uint8_t
{
    NoMsg = 0,
    Waiting = 1,
    Starting = 2,
    Started = 3
};

/**
 * @brief BMS data structure
 */
struct BmsData
{
    uint8_t raw_data[8];
    BmsStatus status;
};

/**
 * @brief Unified page class for all display pages
 * Handles driver telemetry, VCU faults, BMS status, and snake game
 */
class Page
{
public:
    static constexpr uint8_t LCD_WIDTH = 20;
    static constexpr uint8_t LCD_HEIGHT = 4;
    static constexpr uint8_t MAX_SNAKE_LENGTH = 20;

    struct Point
    {
        uint8_t x;
        uint8_t y;
    };

    /**
     * @brief Construct a Page with dependencies
     * @param[in] lcd Reference to the LCD display
     * @param[in] car Reference to the car state
     * @param[in] motor_rpm_ref Reference to motor RPM value
     * @param[in] torque_val_ref Reference to torque value
     * @param[in] motor_warn_ref Reference to motor warning code
     * @param[in] motor_error_ref Reference to motor error code
     * @param[in] odometer_integral_ref Reference to odometer integral
     * @param[in] bms_ref Reference to BMS data
     */
    Page(LiquidCrystal_I2C& lcd,
         CarState& car,
         int16_t& motor_rpm_ref,
         int16_t& torque_val_ref,
         uint16_t& motor_warn_ref,
         uint16_t& motor_error_ref,
         uint32_t& odometer_integral_ref,
         BmsData& bms_ref);

    /**
     * @brief Virtual destructor for proper cleanup
     */
    virtual ~Page() = default;

    /**
     * @brief Setup the page
     * Initializes custom LCD characters and display layout
     */
    void setup();

    /**
     * @brief Update the page
     * Refreshes telemetry displays based on current page
     */
    void update();

    /**
     * @brief Set current page number (0-4)
     * @param[in] page_num Page number to display
     */
    void set_page(uint8_t page_num);

    /**
     * @brief Get current page number
     * @return Current page number
     */
    uint8_t get_page() const { return current_page; }

    /**
     * @brief Handle snake game input
     * @param[in] key Input key ('w', 'a', 's', 'd', 'r')
     */
    void handle_snake_input(char key);

protected:
    LiquidCrystal_I2C& lcd;
    CarState& car;
    int16_t& motor_rpm;
    int16_t& torque_val;
    uint16_t& motor_warn;
    uint16_t& motor_error;
    uint32_t& odometer_integral;
    BmsData& bms;

    uint8_t current_page = 0;
    uint8_t update_state = 0;
    static constexpr uint8_t NUM_UPDATE_STATES = 5;

    // Driver page helpers
    void update_speed();
    void update_rpm();
    void update_throttle();
    void update_motor_status();
    void update_odometer();

    // VCU page helpers
    void vcu_display_header();
    void vcu_display_fault_status();
    bool vcu_initialized = false;

    // BMS page helpers
    void bms_display_header();
    void bms_display_status();
    bool bms_initialized = false;

    // Snake game helpers
    void snake_init_game();
    void snake_spawn_apple();
    void snake_update_game();
    void snake_render_game();
    void snake_check_collision();

    // Snake game data
    Point snake[MAX_SNAKE_LENGTH];
    uint8_t snake_length = 0;
    Point apple;
    uint8_t snake_direction = 1;      // 0=up, 1=right, 2=down, 3=left
    uint8_t snake_next_direction = 1;
    bool snake_game_over = false;
    uint32_t snake_last_move_time = 0;
    bool snake_initialized = false;
    static constexpr uint32_t SNAKE_MOVE_DELAY_MS = 300;

    // Helper functions
    const char* getPedalFaultString(uint8_t fault_byte);
    const char* getBmsStatusString(BmsStatus status);
};

#include "Page.tpp"

#endif // PAGE_HPP

