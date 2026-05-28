/**
 * @file Page.tpp
 * @author chiho
 * @brief Implementation of the unified Page class
 * @version 1.0
 * @date 2026-05-27
 * @see Page.hpp
 */

#include "Page.hpp"
#include <LiquidCrystal_I2C.h>
#include "CarState.hpp"
#include <Arduino.h>

// RPM calculation constants
namespace rpm_calc
{
    constexpr uint32_t ipow(uint32_t base, unsigned exp)
    {
        return exp == 0 ? 1u : base * ipow(base, exp - 1);
    }

    constexpr uint8_t GEAR_RATIO_NUMERATOR = 50;
    constexpr uint8_t GEAR_RATIO_DENOMINATOR = 13;
    constexpr uint16_t WHEEL_DIAMETER_MM = 455;
    constexpr uint16_t MAX_MOTOR_RPM = 7000;
    constexpr uint16_t MAX_MOTOR_RPM_READING = 32767;
    constexpr uint16_t MAX_TORQUE_VAL = 32767;
    constexpr uint32_t MM_PER_KM = (uint32_t)1000 * 1000;
    constexpr uint8_t MINUTES_PER_HOUR = 60;
    constexpr uint16_t SECONDS_PER_HOUR = 3600;
    constexpr uint8_t NUM_DECIMAL_PLACE = 5;
    constexpr uint32_t FIXED_POINT_MULTIPLIER = ipow(10, NUM_DECIMAL_PLACE);
    constexpr double PI_ = 3.1415926535897932384626433832795;

    constexpr uint16_t RPM_TO_KMH_DIVISOR = (double)MAX_TORQUE_VAL / MAX_MOTOR_RPM /
                                               WHEEL_DIAMETER_MM / PI_ * MM_PER_KM / MINUTES_PER_HOUR / 
                                               GEAR_RATIO_DENOMINATOR * GEAR_RATIO_NUMERATOR + 0.5f;
    constexpr uint32_t RPM_INTEGRAL_TO_KM_DIVISOR = (double)MAX_TORQUE_VAL / MAX_MOTOR_RPM /
                                                        WHEEL_DIAMETER_MM / PI_ * MM_PER_KM / MINUTES_PER_HOUR / 
                                                        GEAR_RATIO_DENOMINATOR * GEAR_RATIO_NUMERATOR *
                                                        20 * SECONDS_PER_HOUR / FIXED_POINT_MULTIPLIER + 0.5f;
}

#define char_locked 0

/**
 * @brief Construct a new Page object
 */
inline Page::Page(LiquidCrystal_I2C& lcd_,
                  CarState& car_,
                  int16_t& motor_rpm_ref,
                  int16_t& torque_val_ref,
                  uint16_t& motor_warn_ref,
                  uint16_t& motor_error_ref,
                  uint32_t& odometer_integral_ref,
                  BmsData& bms_ref)
    : lcd(lcd_),
      car(car_),
      motor_rpm(motor_rpm_ref),
      torque_val(torque_val_ref),
      motor_warn(motor_warn_ref),
      motor_error(motor_error_ref),
      odometer_integral(odometer_integral_ref),
      bms(bms_ref)
{
}

/**
 * @brief Setup the page - initializes custom LCD characters
 */
inline void Page::setup()
{
    // Setup custom characters
    byte degCelsius[8] = {
        0b01000,
        0b10100,
        0b01000,
        0b00011,
        0b00100,
        0b00100,
        0b00100,
        0b00011
    };

    byte byte_char_locked[8] = {
        0b01110,
        0b10001,
        0b10001,
        0b11111,
        0b11011,
        0b11011,
        0b11011,
        0b11111
    };

    lcd.createChar(1, degCelsius);
    lcd.createChar(char_locked, byte_char_locked);

    update_state = 0;
}

/**
 * @brief Update the page - cycles through display updates based on current page
 */
inline void Page::update()
{
    switch (current_page)
    {
    case 0: // Driver page
    {
        switch (update_state)
        {
        case 0:
            update_speed();
            break;
        case 1:
            update_rpm();
            break;
        case 2:
            update_throttle();
            break;
        case 3:
            update_motor_status();
            break;
        case 4:
            update_odometer();
            break;
        }
        break;
    }
    case 1: // VCU page
    {
        if (!vcu_initialized)
        {
            vcu_display_header();
            vcu_initialized = true;
        }
        vcu_display_fault_status();
        break;
    }
    case 2: // BMS page
    {
        if (!bms_initialized)
        {
            bms_display_header();
            bms_initialized = true;
        }
        bms_display_status();
        break;
    }
    case 3: // Reserved page
    {
        // Placeholder for future use
        break;
    }
    case 4: // Snake game page
    {
        if (!snake_initialized)
        {
            snake_init_game();
            snake_initialized = true;
        }
        snake_update_game();
        snake_render_game();
        break;
    }
    }

    update_state = (update_state + 1) % NUM_UPDATE_STATES;
}

/**
 * @brief Set current page number
 */
inline void Page::set_page(uint8_t page_num)
{
    if (page_num != current_page)
    {
        current_page = page_num;
        update_state = 0;
        vcu_initialized = false;
        bms_initialized = false;
        snake_initialized = false;
        
        // Page-specific initialization
        if (page_num == 4)
        {
            snake_init_game();
        }
    }
}


/**
 * @brief Update speed display
 */
inline void Page::update_speed()
{
    lcd.setCursor(0, 0);
    uint8_t speed = abs(motor_rpm) / rpm_calc::RPM_TO_KMH_DIVISOR;
    char speed_str[5];
    speed_str[4] = '\0';
    for (int i = 3; i >= 1; --i)
    {
        speed_str[i] = (speed % 10) + '0';
        speed /= 10;
    }
    speed_str[0] = (motor_rpm >= 0) ? '+' : '-';
    lcd.print(speed_str);
}

/**
 * @brief Update RPM display
 */
inline void Page::update_rpm()
{
    lcd.setCursor(11, 0);
    uint16_t rpm = (uint32_t)abs(motor_rpm) * rpm_calc::MAX_MOTOR_RPM / rpm_calc::MAX_MOTOR_RPM_READING;
    char rpm_str[6];
    rpm_str[5] = '\0';
    for (int i = 4; i >= 1; --i)
    {
        rpm_str[i] = (rpm % 10) + '0';
        rpm /= 10;
    }
    rpm_str[0] = (motor_rpm >= 0) ? '+' : '-';
    lcd.print(rpm_str);
}

/**
 * @brief Update throttle display
 */
inline void Page::update_throttle()
{
    lcd.setCursor(14, 1);
    uint8_t throttle_percent = (uint32_t)(abs(torque_val) + 162) * 100 / 32500;
    char throttle_str[5];
    throttle_str[4] = '\0';
    throttle_str[0] = (torque_val >= 0) ? '+' : '-';
    for (int i = 3; i >= 1; --i)
    {
        throttle_str[i] = (throttle_percent % 10) + '0';
        throttle_percent /= 10;
    }
    lcd.print(throttle_str);
}

/**
 * @brief Update motor status display
 */
inline void Page::update_motor_status()
{
    lcd.setCursor(16, 2);
    lcd.print("00");
    lcd.setCursor(16, 2);
    lcd.print(motor_warn, HEX);
    lcd.setCursor(18, 2);
    lcd.print("00");
    lcd.setCursor(18, 2);
    lcd.print(motor_error, HEX);

    // drive mode
    lcd.setCursor(9, 0);
    switch (car.pedal.status.bits.car_status)
    {
    case CarStatus::Init:
        lcd.write(char_locked);
        break;
    case CarStatus::Startin:
        lcd.print("S");
        break;
    case CarStatus::Bussin:
        lcd.print("B");
        break;
    case CarStatus::Drive:
        lcd.print("D");
        break;
    }
}

/**
 * @brief Update odometer display
 */
inline void Page::update_odometer()
{
    constexpr uint8_t ODO_NUM_DIGITS = 6;
    constexpr uint8_t ODO_DECIMAL_PLACES = rpm_calc::NUM_DECIMAL_PLACE;
    constexpr uint8_t ODO_STR_LENGTH = ODO_NUM_DIGITS + 1 + 1;
    constexpr uint8_t ODO_POS_OFFSET = ODO_STR_LENGTH + 1 + 2;
    constexpr uint8_t STR_START_POS = 21 - ODO_POS_OFFSET;
    
    lcd.setCursor(STR_START_POS, 3);
    uint32_t odometer = odometer_integral / rpm_calc::RPM_INTEGRAL_TO_KM_DIVISOR;
    char odometer_str[ODO_STR_LENGTH];
    odometer_str[ODO_STR_LENGTH - 1] = '\0';
    for (int i = ODO_STR_LENGTH - 2; i >= 0; --i)
    {
        if (i == ODO_NUM_DIGITS - ODO_DECIMAL_PLACES)
        {
            odometer_str[i] = '.';
        }
        else
        {
            odometer_str[i] = (odometer % 10) + '0';
            odometer /= 10;
        }
    }
    lcd.print(odometer_str);
}


/**
 * @brief Display VCU page header
 */
inline void Page::vcu_display_header()
{
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("  VCU/CAR PROBLEMS");
    lcd.setCursor(0, 1);
    lcd.print("Pedal Faults:");
}

/**
 * @brief Display VCU fault status
 */
inline void Page::vcu_display_fault_status()
{
    if (update_state >= 0 && update_state <= 3)
    {
        lcd.setCursor(0, 2 + update_state);
        lcd.print("                    ");
        lcd.setCursor(0, 2 + update_state);

        if (car.pedal.faults.byte == 0)
        {
            if (update_state == 0)
                lcd.print("OK");
        }
        else
        {
            uint8_t fault_index = update_state % 8;
            if (car.pedal.faults.byte & (1 << fault_index))
            {
                lcd.print(getPedalFaultString(car.pedal.faults.byte));
            }
        }
    }
}


/**
 * @brief Display BMS page header
 */
inline void Page::bms_display_header()
{
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("   BMS PROBLEMS");
    lcd.setCursor(0, 1);
    lcd.print("BMS Status:");
}

/**
 * @brief Display BMS status
 */
inline void Page::bms_display_status()
{
    if (update_state >= 0 && update_state <= 2)
    {
        lcd.setCursor(0, 2);
        lcd.print(" ");
        lcd.setCursor(0, 2);
        lcd.print(getBmsStatusString(bms.status));

        lcd.setCursor(0, 3);
        lcd.print(" ");
        if (bms.status == BmsStatus::NoMsg)
        {
            lcd.setCursor(0, 3);
            lcd.print("ERROR: No BMS msg!");
        }
    }
}


/**
 * @brief Initialize the snake game
 */
inline void Page::snake_init_game()
{
    snake_length = 3;
    snake[0] = {10, 2};
    snake[1] = {9, 2};
    snake[2] = {8, 2};
    snake_direction = 1; // moving right
    snake_next_direction = 1;
    snake_game_over = false;
    snake_last_move_time = millis();
    snake_spawn_apple();
    lcd.clear();
}

/**
 * @brief Spawn a new apple in the snake game
 */
inline void Page::snake_spawn_apple()
{
    bool valid = false;
    while (!valid)
    {
        apple.x = random(0, LCD_WIDTH);
        apple.y = random(0, LCD_HEIGHT);
        valid = true;
        for (uint8_t i = 0; i < snake_length; ++i)
        {
            if (snake[i].x == apple.x && snake[i].y == apple.y)
            {
                valid = false;
                break;
            }
        }
    }
}

/**
 * @brief Update the snake game state
 */
inline void Page::snake_update_game()
{
    if (snake_game_over)
        return;

    uint32_t now = millis();
    if (now - snake_last_move_time < SNAKE_MOVE_DELAY_MS)
        return;

    snake_direction = snake_next_direction;
    Point new_head = snake[0];
    
    switch (snake_direction)
    {
    case 0: // up
        new_head.y = (new_head.y == 0) ? LCD_HEIGHT - 1 : new_head.y - 1;
        break;
    case 1: // right
        new_head.x = (new_head.x == LCD_WIDTH - 1) ? 0 : new_head.x + 1;
        break;
    case 2: // down
        new_head.y = (new_head.y == LCD_HEIGHT - 1) ? 0 : new_head.y + 1;
        break;
    case 3: // left
        new_head.x = (new_head.x == 0) ? LCD_WIDTH - 1 : new_head.x - 1;
        break;
    }

    snake_check_collision();

    // Move snake
    for (uint8_t i = snake_length; i > 0; --i)
    {
        snake[i] = snake[i - 1];
    }
    snake[0] = new_head;
    
    if (new_head.x == apple.x && new_head.y == apple.y)
    {
        if (snake_length < MAX_SNAKE_LENGTH)
        {
            snake_length++;
        }
        snake_spawn_apple();
    }

    snake_last_move_time = now;
}

/**
 * @brief Check for collisions in the snake game
 */
inline void Page::snake_check_collision()
{
    Point new_head = snake[0];
    for (uint8_t i = 0; i < snake_length; ++i)
    {
        if (new_head.x == snake[i].x && new_head.y == snake[i].y)
        {
            snake_game_over = true;
            return;
        }
    }
}

/**
 * @brief Render the snake game
 */
inline void Page::snake_render_game()
{
    lcd.clear();
    for (uint8_t i = 0; i < snake_length; ++i)
    {
        lcd.setCursor(snake[i].x, snake[i].y);
        if (i == 0)
            lcd.print("@"); // head
        else
            lcd.print("o"); // body
    }
    lcd.setCursor(apple.x, apple.y);
    lcd.print("*");
    
    if (snake_game_over)
    {
        lcd.setCursor(0, 0);
        lcd.print("GAME OVER! Len:");
        lcd.setCursor(15, 0);
        lcd.print(snake_length, DEC);
        lcd.setCursor(2, 2);
        lcd.print("Press R to restart");
    }
}

/**
 * @brief Handle snake game input
 */
inline void Page::handle_snake_input(char key)
{
    if (current_page != 4)
        return;

    switch (key)
    {
    case 'w': // up
        if (snake_direction != 2)
            snake_next_direction = 0;
        break;
    case 'd': // right
        if (snake_direction != 3)
            snake_next_direction = 1;
        break;
    case 's': // down
        if (snake_direction != 0)
            snake_next_direction = 2;
        break;
    case 'a': // left
        if (snake_direction != 1)
            snake_next_direction = 3;
        break;
    case 'r': // restart
        snake_init_game();
        snake_initialized = true;
        break;
    }
}

/**
 * @brief Get pedal fault string from fault byte
 */
inline const char* Page::getPedalFaultString(uint8_t fault_byte)
{
    if (fault_byte & 0x01)
        return "Pedal FAULT";
    if (fault_byte & 0x02)
        return "Pedal fault >100ms";
    if (fault_byte & 0x04)
        return "APPS 5V low";
    if (fault_byte & 0x08)
        return "APPS 5V high";
    if (fault_byte & 0x10)
        return "APPS 3V3 low";
    if (fault_byte & 0x20)
        return "APPS 3V3 high";
    if (fault_byte & 0x40)
        return "Brake low";
    if (fault_byte & 0x80)
        return "Brake high";
    return "No fault";
}

/**
 * @brief Get BMS status string
 */
inline const char* Page::getBmsStatusString(BmsStatus status)
{
    switch (status)
    {
    case BmsStatus::NoMsg:
        return "No BMS message";
    case BmsStatus::Waiting:
        return "BMS Waiting";
    case BmsStatus::Starting:
        return "BMS Starting HV";
    case BmsStatus::Started:
        return "BMS HV Started";
    default:
        return "BMS Unknown";
    }
}


