/**
 * @brief MCP2515 instance indices.
 *
 * Used to identify different MCP2515 CAN controller instances.
 */
enum class McpIndex : uint8_t
{
    Vcu = 0,   /**< VCU CAN MCP2515 instance */
    Ssru = 1,  /**< SSRU CAN MCP2515 instance */
    Radio = 2, /**RF Serial, not MCP2515, fix later */
    Screen = 3 /** I2C LCD Screen, not MCP2515, fix later */
};
