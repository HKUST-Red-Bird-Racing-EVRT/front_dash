/**
 * @file DriverPage.tpp
 * @author ChiHo Ngan, Red Bird Racing
 * @brief Driver page.
 * @version 1.1
 * @date 2026-09-30
 * @see Page.hpp, Page.tpp
 */

/** @brief Main page: speed, RPM, odometer, pack current bar and car status. */
class DriverPage : public Page
{
public:
    void setup() override;
    void update(uint8_t item) override;

private:
    void updatepwr();      ///< Pack current bar
    void updaterpm();      ///< Motor RPM
    void update_car_spd(); ///< Speed in km/h
    void updateodo();      ///< Odometer in km
};

void DriverPage::updatepwr(){
	// Sign at col 8, 9-cell magnitude bar at cols 9-17.
	int16_t current = static_cast<int16_t>(static_cast<uint16_t>(bms.raw_data[BMS_BYTE_CURRENT]) |
	                                        (static_cast<uint16_t>(bms.raw_data[BMS_BYTE_CURRENT + 1]) << 8));
	bool regen = current < 0;
	uint16_t mag = absU16(current);
	if (mag > MAX_PACK_CURRENT_DECIAMP)
	{
		mag = MAX_PACK_CURRENT_DECIAMP;
	}
	uint8_t bars = static_cast<uint8_t>(static_cast<uint32_t>(mag) * 9 / MAX_PACK_CURRENT_DECIAMP);
	lcd.setCursorQueued(8, 1);
	lcd.writeQueued(regen ? '-' : '+');
	for (uint8_t i = 0; i < 9; ++i)
	{
		lcd.writeQueued(i < bars ? CHAR_PWR : ' ');
	}
}

void DriverPage::updaterpm(){
	lcd.setCursorQueued(4,1);
	uint16_t rpm = (uint32_t)absU16(motor_rpm) * rpm_calc::MAX_MOTOR_RPM / rpm_calc::MAX_MOTOR_RPM_READING;
	char rpm_str[5];
	rpm_str[4] = '\0';
	for (int8_t i = 3; i >= 0; --i)
	{
		rpm_str[i] = (rpm % 10) + '0';
		rpm /= 10;
	}
	lcd.printQueued(rpm_str);
}

void DriverPage::update_car_spd(){
	lcd.setCursorQueued(4,0);
	uint8_t speed = absU16(motor_rpm) / rpm_calc::RPM_TO_KMH_DIVISOR;
	char speed_str[5];
	speed_str[0] = ' ';
	speed_str[4] = '\0';
	for (uint8_t i = 3; i >= 1; --i)
	{
		speed_str[i] = (speed % 10) + '0';
		speed /= 10;
	}
	lcd.printQueued(speed_str);
}

void DriverPage::updateodo(){
	lcd.setCursorQueued(0, 3);
	uint32_t odometer = odometer_integral / rpm_calc::RPM_INTEGRAL_TO_KM_DIVISOR;
	char odometer_str[ODO_STR_LENGTH];
	odometer_str[ODO_STR_LENGTH - 1] = '\0';
	for (int8_t i = ODO_STR_LENGTH - 2; i >= 0; --i)
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
	lcd.printQueued(odometer_str);
}

/** @brief Draw the driver page labels. */
void DriverPage::setup()
{
    lcd.setCursor(0, 0);
	lcd.print(F("kmh:"));
	lcd.setCursor(0, 1);
	lcd.print(F("rpm:"));
	lcd.setCursor(7, 3);
	lcd.print(F("km"));
	lcd.setCursor(17,3);
	lcd.print(F("%"));
}

/**
 * @brief Redraw one driver page field.
 * @param item 0 car status, 1 odometer, 2 speed, 3 RPM, 4 pack current.
 */
void DriverPage::update(uint8_t item)
{
	switch (item)
	{
	case 0:
		lcd.setCursorQueued(9, 0);
		if (stale(Src::Pedal))
		{
			lcd.writeQueued('?');
			break;
		}
		switch (car.pedal.status.bits.car_status)
		{
		case CarStatus::Init:
			lcd.writeQueued(CHAR_LOCKED);
			break;
		case CarStatus::Startin:
			lcd.writeQueued('S');
			break;
		case CarStatus::Bussin:
			lcd.writeQueued('B');
			break;
		case CarStatus::Drive:
			lcd.writeQueued('D');
			break;
		}
		break;
	case 1:
		updateodo();
		break;
	case 2:
		if (stale(Src::Motor))
		{
			lcd.setCursorQueued(4, 0);
			lcd.printQueued(F("  --"));
		}
		else
			update_car_spd();
		break;
	case 3:
		if (stale(Src::Motor))
		{
			lcd.setCursorQueued(4, 1);
			lcd.printQueued(F("  --"));
		}
		else
			updaterpm();
		break;
	case 4:
		if (stale(Src::Bms))
		{
			lcd.setCursorQueued(8, 1);
			lcd.printQueued(F("  no BMS  "));
		}
		else
			updatepwr();
		break;
	}
}
