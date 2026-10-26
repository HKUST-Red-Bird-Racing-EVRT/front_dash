/**
 * @file BMSPage.tpp
 * @author ChiHo Ngan, Red Bird Racing
 * @brief BMS debug page.
 * @version 1.1
 * @date 2026-09-30
 * @see Page.hpp, Page.tpp
 */

/** @brief BMS debug page: fault byte, temperature, status, max/min cell voltage. */
class BMSPage : public Page
{
public:
    void setup() override;
    void update(uint8_t item) override;
};

/** @brief Draw the BMS page labels. */
void BMSPage::setup()
{
	lcd.setCursor(7,0);
	lcd.print(F("BMS"));
    lcd.setCursor(0, 1);
    lcd.print(F("ERROR:"));
	lcd.setCursor(3,2);
	lcd.write(CHAR_DEG);
    lcd.setCursor(0, 3);
    lcd.print(F("MAX:"));
	lcd.setCursor(7,3);
	lcd.print(F("V"));
	lcd.setCursor(10, 3);
	lcd.print(F("MIN:"));
	lcd.setCursor(17, 3);
	lcd.print(F("V"));
}

/**
 * @brief Redraw one BMS page field.
 * @param item 0 faults, 1 temperature, 2 status, 3 max voltage, 4 min voltage.
 */
void BMSPage::update(uint8_t item)
{
	char buf[4];
	switch (item)
	{
	case 0:
	{
		uint8_t status_fault = bms.raw_data[BMS_BYTE_STATUS];
		lcd.setCursorQueued(6, 1);
		if (stale(Src::Bms))
		{
			lcd.printQueued(F("--"));
		}
		else if (status_fault == 0)
		{
			lcd.printQueued(F("OK"));
		}
		else
		{
			formatHexByte(status_fault, buf);
			lcd.printQueued(buf);
		}
		break;
	}
	case 1:
		lcd.setCursorQueued(0, 2);
		if (stale(Src::Bms))
		{
			lcd.printQueued(F(" --"));
			break;
		}
		formatSignedInt2(static_cast<int8_t>(bms.raw_data[BMS_BYTE_TEMP]), buf);
		lcd.printQueued(buf);
		break;
	case 2:
		lcd.setCursorQueued(5, 2);
		if (stale(Src::Bms))
		{
			lcd.printQueued(F("N/A"));
			break;
		}
		switch (bms.status)
		{
		case BmsStatus::NoMsg:
			lcd.printQueued(F("N/A"));
			break;
		case BmsStatus::Waiting:
			lcd.printQueued(F("W  "));
			break;
		case BmsStatus::Starting:
			lcd.printQueued(F("S  "));
			break;
		case BmsStatus::Started:
			lcd.printQueued(F("R  "));
			break;
		default:
			lcd.printQueued(F("?  "));
			break;
		}
		break;
	case 3:
	case 4:
	{
		uint8_t at = item == 3 ? BMS_BYTE_VOLTAGE_MAX : BMS_BYTE_VOLTAGE_MIN;
		lcd.setCursorQueued(item == 3 ? 4 : 14, 3);
		if (stale(Src::Bms))
		{
			lcd.printQueued(F("-.-"));
			break;
		}
		formatFixedPoint1(static_cast<uint16_t>(bms.raw_data[at]) | (static_cast<uint16_t>(bms.raw_data[at + 1]) << 8), buf);
		lcd.printQueued(buf);
		break;
	}
	}
}
