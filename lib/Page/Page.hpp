/**
 * @file Page.hpp
 * @author ChiHo
 * @brief Abstract Base Class for display pages
 * @version 1.0
 * @date 2026-05-28
 * @see Page.tpp
 * @dir Page @brief The Page library provides an abstract base class for implementing different display pages on the dashboard. All pages inherit from this base class and must implement setup() and update() methods for page lifecycle management.
 */

#ifndef PAGE_HPP
#define PAGE_HPP

#include <stdint.h>
#include <LiquidCrystal_I2C.h>


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
 * @brief Driver page (main dashboard).
 * Displays main vehicle information (speed, RPM, throttle, etc).
 */
class DashState;
class DriverPage : public Page
{
public:
    /**
     * @brief Constructor for DriverMenuPage.
     * @param lcd Reference to LiquidCrystal_I2C display object.
     * @param state Reference to DashState for vehicle data.
     */
    DriverPage(LiquidCrystal_I2C& lcd, DashState& state);

    /**
     * @brief Setup the driver menu page.
     */
    void setup() override;

    /**
     * @brief Update the driver menu page.
     */
    void update() override;

private:
    LiquidCrystal_I2C& lcd;
    DashState& state;
};

/**
 * @brief VCU Debug page.
 * Displays VCU (Vehicle Control Unit) debug information and diagnostics.
 */
class VCUPage : public Page
{
public:
    /**
     * @brief Constructor for VCUDebug.
     * @param lcd Reference to LiquidCrystal_I2C display object.
     * @param state Reference to DashState for vehicle data.
     */
    VCUPage(LiquidCrystal_I2C& lcd, DashState& state);

    /**
     * @brief Setup the VCU debug.
     */
    void setup() override;

    /**
     * @brief Update the VCU debug.
     */
    void update() override;

private:
    LiquidCrystal_I2C& lcd;
    DashState& state;
};

/**
 * @brief BMS Debug page.
 * Displays BMS (Battery Management System) debug information and diagnostics.
 */
class BMSPage : public Page
{
public:
    /**
     * @brief Constructor for BMSDebug.
     * @param lcd Reference to LiquidCrystal_I2C display object.
     * @param state Reference to DashState for vehicle data.
     */
    BMSPage(LiquidCrystal_I2C& lcd, DashState& state);

    /**
     * @brief Setup the BMS debug.
     */
    void setup() override;

    /**
     * @brief Update the BMS debug.
     */
    void update() override;

private:
    LiquidCrystal_I2C& lcd;
    DashState& state;
};

/**
 * @brief Reserved page.
 * Placeholder
 */
class ReservedPage : public Page
{
public:
    /**
     * @brief Constructor. 
     * @param lcd Reference to LiquidCrystal_I2C display object.
     * @param state Reference to DashState for vehicle data.
     */
    ReservedPage(LiquidCrystal_I2C& lcd, DashState& state);

    /**
     * @brief Setup.
     */
    void setup() override;

    /**
     * @brief Update.
     */
    void update() override;

private:
    LiquidCrystal_I2C& lcd;
    DashState& state;
};

#include "Page.tpp" // implementation

#endif // PAGE_HPP