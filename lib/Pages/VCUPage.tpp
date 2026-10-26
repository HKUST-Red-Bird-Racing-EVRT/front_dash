/**
 * @file VCUPage.tpp
 * @author ChiHo Ngan, Red Bird Racing
 * @brief VCU debug page.
 * @version 1.1
 * @date 2026-09-30
 * @see Page.hpp, Page.tpp
 */

/** @brief VCU debug page: motor error word, car state, POT and throttle. */
class VCUPage : public Page
{
public:
    void setup() override;
    void update(uint8_t item) override;
};

/** @brief Draw the VCU page labels. */
void VCUPage::setup()
{
	lcd.setCursor(7,0);
	lcd.print(F("VCU"));
    lcd.setCursor(0, 1);
    lcd.print(F("ERROR:"));
    lcd.setCursor(0, 2);
    lcd.print(F("STATE:"));
	lcd.setCursor(0,3);
	lcd.print(F("POT:"));
	lcd.setCursor(9,3);
	lcd.print(F("PEDAL:"));
}

/**
 * @brief Redraw one VCU page field.
 * @param item 0 motor error, 1 car state, 2 status byte, 3 POT, 4 throttle.
 */
void VCUPage::update(uint8_t item)
{
	char buf[5];
	switch (item)
	{
	case 0:
		lcd.setCursorQueued(6, 1);
		if (stale(Src::Motor))
		{
			lcd.printQueued(F("----"));
			break;
		}
		formatHexWord(motor_error, buf);
		lcd.printQueued(buf);
		break;
	case 1:
		lcd.setCursorQueued(6, 2);
		if (stale(Src::Pedal))
			lcd.printQueued(F("N/A  "));
		else
			printCarStatusQueued(car.pedal.status.bits.car_status);
		break;
	case 2:
		lcd.setCursorQueued(12, 2);
		if (stale(Src::Pedal))
		{
			lcd.printQueued(F("--"));
			break;
		}
		formatHexByte(car.pedal.status.byte, buf);
		lcd.printQueued(buf);
		break;
	case 3:
		lcd.setCursorQueued(4, 3);
		formatHexWord(static_cast<uint16_t>(pot_buffer[0]) | (static_cast<uint16_t>(pot_buffer[1]) << 8), buf);
		lcd.printQueued(buf);
		break;
	case 4:
		if (stale(Src::Pedal))
		{
			lcd.setCursorQueued(15, 3);
			lcd.printQueued(F("---"));
		}
		else
			updatethrottle(15, 3);
		break;
	}
}
