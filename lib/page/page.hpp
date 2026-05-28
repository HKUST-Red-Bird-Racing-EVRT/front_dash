/**
 * @file Page.hpp
 * @author Red Bird Racing
 * @brief Abstract Base Class for display pages
 * @version 1.0
 * @date 2026-05-23
 * @see Page.tpp
 * @dir Page @brief The Page library provides an abstract base class for implementing different display pages on the dashboard. All pages inherit from this base class and must implement setup() and update() methods for page lifecycle management.
 */

#ifndef PAGE_HPP
#define PAGE_HPP

#include <stdint.h>

class LiquidCrystal_I2C;

/**
 * @brief Abstract Base Class for display pages.
 * Defines the common interface for all pages on the dashboard.
 * Each page must implement setup() for initialization and update() for continuous updates.
 */
class Page
{
public:
    /**
     * @brief Setup the page.
     * Called once when the page is activated/switched to.
     * Derived classes should initialize page-specific resources here.
     */
    virtual void setup() = 0;

    /**
     * @brief Update the page.
     * Called repeatedly in the main loop while the page is active.
     * Derived classes should update display and handle logic here.
     */
    virtual void update() = 0;
};

/**
 * @brief Dashboard page implementation.
 * Displays main vehicle information (speed, RPM, throttle, etc).
 */
class DashState;
class DashboardPage : public Page
{
public:
    /**
     * @brief Constructor for DashboardPage.
     * @param lcd Pointer to LiquidCrystal_I2C display object.
     */
    DashboardPage(LiquidCrystal_I2C& lcd, DashState& state);

    /**
     * @brief Setup the dashboard page.
     */
    void setup() override;

    /**
     * @brief Update the dashboard page.
     */
    void update() override;

private:
    LiquidCrystal_I2C& lcd;
    DashState& state;
};

#include "Page.tpp" // implementation

#endif // PAGE_HPP