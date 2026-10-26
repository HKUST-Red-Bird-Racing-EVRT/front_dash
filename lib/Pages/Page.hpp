/**
 * @file Page.hpp
 * @author ChiHo Ngan, Red Bird Racing
 * @brief Page interface and all concrete pages.
 * @details Pages draw through the global `lcd` and read the global telemetry.
 * @version 2.0
 * @date 2026-09-30
 * @see Page.tpp
 */

#ifndef PAGE_HPP
#define PAGE_HPP

#include "namespace.h"
#include <stdint.h>
#include "Dash_I2C.hpp"

/** @brief Base class for an LCD page. */
class Page
{
public:
    /** @brief Draw the static labels. Called after the LCD is cleared. */
    virtual void setup() = 0;

    /**
     * @brief Redraw one field. Called every lcd_update::update_interval_ms.
     * @param item Field index, 0 to lcd_update::update_items - 1.
     * @attention Must not change state: it is called twice per tick (dry run, then draw).
     */
    virtual void update(uint8_t item) = 0;
};

#include "Page.tpp"
#include "DriverPage.tpp"
#include "VCUPage.tpp"
#include "BMSPage.tpp"
#include "DefaultPage.tpp"

#endif // PAGE_HPP
