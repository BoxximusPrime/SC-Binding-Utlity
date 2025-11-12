/**
 * Shared utilities for handling joystick input detection and formatting
 */

import { convertToSCAxisFormat, shouldInvertAxis, getAxisMapping } from './axis-mapping.js';

/**
 * Parse a Star Citizen input string and return a friendly display name
 * @param {string} inputString - SC format like "js1_button3", "gp1_button3", "js1_hat1_up", "js2_axis1", "js2_axis1_positive"
 * @returns {string} - Friendly name like "Button 3", "Hat 1 Up", "Axis 1", "Axis 1 +"
 */
export function parseInputDisplayName(inputString)
{
    if (!inputString) return '';

    // Hat switch: js1_hat1_up or gp1_hat1_up -> "Hat 1 Up"
    if (inputString.includes('_hat'))
    {
        const hatMatch = inputString.match(/hat(\d+)_(\w+)/);
        if (hatMatch)
        {
            const hatNum = hatMatch[1];
            const direction = hatMatch[2].charAt(0).toUpperCase() + hatMatch[2].slice(1);
            return `Hat ${hatNum} ${direction}`;
        }
    }

    // Button: js1_button3 or gp1_button3 -> "Button 3"
    if (inputString.includes('_button'))
    {
        const btnMatch = inputString.match(/button(\d+)/);
        if (btnMatch)
        {
            return `Button ${btnMatch[1]}`;
        }
    }

    // Axis with direction: js1_axis1_positive or gp1_axis1_positive -> "Axis 1 +"
    // Axis without direction: js1_axis1 or gp1_axis1 -> "Axis 1"
    if (inputString.includes('_axis'))
    {
        const axisMatch = inputString.match(/axis(\d+)(?:_(positive|negative))?/);
        if (axisMatch)
        {
            const axisNum = axisMatch[1];
            const direction = axisMatch[2];
            if (direction)
            {
                const symbol = direction === 'positive' ? '+' : '-';
                return `Axis ${axisNum} ${symbol}`;
            }
            return `Axis ${axisNum}`;
        }
    }

    // Fallback to the original string
    return inputString;
}

/**
 * Parse a Star Citizen input string and return a short display name
 * @param {string} inputString - SC format like "js1_button3", "js1_hat1_up", "js2_axis1", "js2_axis1_positive"
 * @returns {string} - Short name like "Btn 3", "Hat 1 Up", "Axis 1", "Axis 1 +"
 */
export function parseInputShortName(inputString)
{
    if (!inputString) return '';

    // Hat switch: js1_hat1_up -> "Hat 1 Up"
    if (inputString.includes('_hat'))
    {
        const hatMatch = inputString.match(/hat(\d+)_(\w+)/);
        if (hatMatch)
        {
            const hatNum = hatMatch[1];
            const direction = hatMatch[2].charAt(0).toUpperCase() + hatMatch[2].slice(1);
            return `Hat ${hatNum} ${direction}`;
        }
    }

    // Button: js1_button3 -> "Btn 3"
    if (inputString.includes('_button'))
    {
        const btnMatch = inputString.match(/button(\d+)/);
        if (btnMatch)
        {
            return `Btn ${btnMatch[1]}`;
        }
    }

    // Axis with direction: js1_axis1_positive -> "Axis 1 +"
    // Axis without direction: js1_axis1 -> "Axis 1"
    if (inputString.includes('_axis'))
    {
        const axisMatch = inputString.match(/axis(\d+)(?:_(positive|negative))?/);
        if (axisMatch)
        {
            const axisNum = axisMatch[1];
            const direction = axisMatch[2];
            if (direction)
            {
                const symbol = direction === 'positive' ? '+' : '-';
                return `Axis ${axisNum} ${symbol}`;
            }
            return `Axis ${axisNum}`;
        }
    }

    // Fallback to the original string
    return inputString;
}

/**
 * Get the input type from a Star Citizen input string
 * @param {string} inputString - SC format like "js1_button3", "gp1_button3", "js1_hat1_up", "js2_axis1", "kb1_w"
 * @returns {string} - Type: "button", "hat", "axis", "keyboard", or "unknown"
 */
export function getInputType(inputString)
{
    if (!inputString) return 'unknown';

    if (inputString.includes('_hat')) return 'hat';
    if (inputString.includes('_button')) return 'button';
    if (inputString.includes('_axis')) return 'axis';
    if (inputString.startsWith('kb1_')) return 'keyboard';
    if (inputString.startsWith('gp1_') || inputString.startsWith('js1_')) return 'gamepad'; // Both are game controllers

    return 'unknown';
}

/**
 * Get the joystick/gamepad instance number from a Star Citizen input string
 * @param {string} inputString - SC format like "js1_button3", "gp1_button3", "js2_hat1_up"
 * @returns {number} - Device instance (1, 2, etc.) or 0 if not found
 */
export function getJoystickInstance(inputString)
{
    if (!inputString) return 0;

    // Match both js and gp prefixes
    const match = inputString.match(/(?:js|gp)(\d+)_/);
    return match ? parseInt(match[1]) : 0;
}

/**
 * Get the axis direction from a Star Citizen axis input string
 * @param {string} inputString - SC format like "js1_axis1_positive", "gp1_axis1_negative"
 * @returns {string|null} - "positive", "negative", or null if not an axis with direction
 */
export function getAxisDirection(inputString)
{
    if (!inputString || !inputString.includes('_axis')) return null;

    const match = inputString.match(/axis\d+_(positive|negative)/);
    return match ? match[1] : null;
}

/**
 * Get the axis number from a Star Citizen axis input string
 * @param {string} inputString - SC format like "js1_axis1", "gp1_axis1", or "js1_axis1_positive"
 * @returns {number} - Axis number or 0 if not found
 */
export function getAxisNumber(inputString)
{
    if (!inputString || !inputString.includes('_axis')) return 0;

    const match = inputString.match(/axis(\d+)/);
    return match ? parseInt(match[1]) : 0;
}

/**
 * Process a detected input result from the backend
 * @param {Object} result - Result from wait_for_input_binding
 * @returns {Object} - Processed result with additional helper properties
 */
export function processDetectedInput(result)
{
    if (!result) return null;

    const processed = {
        ...result,
        friendlyName: parseInputDisplayName(result.input_string),
        shortName: parseInputShortName(result.input_string),
        type: getInputType(result.input_string),
        joystickInstance: getJoystickInstance(result.input_string)
    };

    // Add axis-specific properties if this is an axis input
    if (processed.type === 'axis')
    {
        processed.axisDirection = getAxisDirection(result.input_string);
        processed.axisNumber = getAxisNumber(result.input_string);

        // Convert to Star Citizen format
        const jsInstance = processed.joystickInstance;
        const mapping = getAxisMapping(jsInstance);
        processed.scFormat = convertToSCAxisFormat(result.input_string, mapping);
        processed.shouldInvert = shouldInvertAxis(result.input_string);
    }

    return processed;
}

/**
 * Convert any input string to Star Citizen format
 * For axes: converts "js1_axis3_positive" or "gp1_axis3_positive" to "js1_z" or "gp1_z"
 * For buttons/hats: returns unchanged
 * @param {string} inputString - Input string in any format
 * @returns {string} - Star Citizen compatible format
 */
export function toStarCitizenFormat(inputString)
{
    const type = getInputType(inputString);

    if (type === 'axis')
    {
        const jsInstance = getJoystickInstance(inputString);
        const mapping = getAxisMapping(jsInstance);
        return convertToSCAxisFormat(inputString, mapping);
    }

    // Buttons and hats are already in correct format (both js and gp prefixes are valid)
    return inputString;
}
