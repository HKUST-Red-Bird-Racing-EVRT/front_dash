/**
 * @file DefaultPage.tpp
 * @author ChiHo Ngan, Red Bird Racing
 * @brief Status page.
 * @version 1.1
 * @date 2026-09-30
 * @see Page.hpp, Page.tpp
 */

/** @brief Status page: brake fault, car state, HV ready and pedal fault summary. */
class DefaultPage : public Page
{
public:
    void setup() override;
    void update(uint8_t item) override;
};

/** @brief Draw the status page labels. */
void DefaultPage::setup()
{
    lcd.setCursor(0, 0);
    lcd.print(F("BRAKE:"));
    lcd.setCursor(0, 1);
    lcd.print(F("BUTTON:"));
    lcd.setCursor(0, 2);
    lcd.print(F("HV:"));
}

/**
 * @brief Redraw one status page field.
 * @param item 0 brake fault, 1 car status, 2 HV ready, 3 fault summary.
 */
void DefaultPage::update(uint8_t item)
{
	if (stale(Src::Pedal))
	{
		switch (item)
		{
		case 0:
			lcd.setCursorQueued(6, 0);
			lcd.printQueued(F("N/A "));
			break;
		case 1:
			lcd.setCursorQueued(7, 1);
			lcd.printQueued(F("N/A  "));
			break;
		case 2:
			lcd.setCursorQueued(3, 2);
			lcd.writeQueued('?');
			break;
		case 3:
			lcd.setCursorQueued(0, 3);
			lcd.printQueued(F("no VCU data       "));
			break;
		}
		return;
	}
	switch (item)
	{
	case 0:
		lcd.setCursorQueued(6, 0);
		if (car.pedal.faults.bits.brake_high)
			lcd.printQueued(F("HIGH"));
		else if (car.pedal.faults.bits.brake_low)
			lcd.printQueued(F("LOW "));
		else
			lcd.printQueued(F("OK  "));
		break;
	case 1:
		lcd.setCursorQueued(7, 1);
		printCarStatusQueued(car.pedal.status.bits.car_status);
		break;
	case 2:
		lcd.setCursorQueued(3, 2);
		lcd.writeQueued(car.pedal.status.bits.hv_ready ? 'Y' : 'N');
		break;
	case 3:
		lcd.setCursorQueued(0, 3);
		lcd.printQueued(car.pedal.faults.bits.fault_active ? F("Pedal/Brake error ") : F("no Error          "));
		break;
	}
}
