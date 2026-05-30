#include "Enums.hpp"
#include "Page.hpp"
#include "CarState.hpp"

// BMS State
struct BmsData
{
	uint8_t raw_data[8];
	BmsStatus status;
} bms;


// Page instance
DashboardPage dashboardPage(lcd);
Page* currentPage = &dashboardPage;

CarState car;
Page page(lcd, car, motor_rpm, torque_val, motor_warn, motor_error, odometer_integral, bms);



// Snake Game State
struct SnakeGame
{
	static constexpr uint8_t LCD_WIDTH = 20;
	static constexpr uint8_t LCD_HEIGHT = 4;
	static constexpr uint8_t MAX_SNAKE_LENGTH = 20;
	struct Point
	{
		uint8_t x;
		uint8_t y;
	};

	Point snake[MAX_SNAKE_LENGTH];
	uint8_t snake_length;
	Point apple;
	uint8_t direction; // 0=up, 1=right, 2=down, 3=left
	uint8_t next_direction;
	bool game_over;
	uint32_t last_move_time;
	static constexpr uint32_t MOVE_DELAY_MS = 300;

	void init()
	{
		snake_length = 3;
		snake[0] = {10, 2};
		snake[1] = {9, 2};
		snake[2] = {8, 2};
		direction = 1; // moving right
		next_direction = 1;
		game_over = false;
		last_move_time = millis();
		spawn_apple();
	}

	void spawn_apple()
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

	void update()
	{
		if (game_over)
			return;

		uint32_t now = millis();
		if (now - last_move_time < MOVE_DELAY_MS)
			return;

		direction = next_direction;
		Point new_head = snake[0];
		switch (direction)
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
		// Check collision
		for (uint8_t i = 0; i < snake_length; ++i)
		{
			if (new_head.x == snake[i].x && new_head.y == snake[i].y)
			{
				game_over = true;
				return;
			}
		}

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
			spawn_apple();
		}

		last_move_time = now;
		delay(50);
	}
} snake_game;