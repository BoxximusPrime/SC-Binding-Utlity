// ============================================================================
// Shared Button Rendering Utilities
// ============================================================================
// This module provides consistent button and box rendering functions
// for both the template editor and joystick viewer.

// ========================================
// Constants
// ========================================

export const ButtonFrameWidth = 220;
export const ButtonFrameHeight = 120;
export const HatFrameWidth = 140;
export const HatFrameHeight = 100;
export const HatButtonGap = 4;
export const NumLines = 6;
export const HatSpacing = 6;
export const ActionTextSize = '14px';

// Font sizes
export const TitleFontSize = '16px';
export const TitleFontSizeCompact = '11px';
export const ContentFontSize = '14px';
export const ContentFontSizeCompact = '14px';
export const HatTitleFontSize = '16px';
export const TextLineHeight = 12;
export const TextLineHeightCompact = 11;

// ========================================
// Drawing Helper Functions
// ========================================

/**
 * Draw a rounded rectangle path
 */
export function roundRect(ctx, x, y, width, height, radius)
{
    ctx.beginPath();
    ctx.moveTo(x + radius, y);
    ctx.lineTo(x + width - radius, y);
    ctx.quadraticCurveTo(x + width, y, x + width, y + radius);
    ctx.lineTo(x + width, y + height - radius);
    ctx.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
    ctx.lineTo(x + radius, y + height);
    ctx.quadraticCurveTo(x, y + height, x, y + height - radius);
    ctx.lineTo(x, y + radius);
    ctx.quadraticCurveTo(x, y, x + radius, y);
    ctx.closePath();
}

/**
 * Simplify button names for display
 * Removes prefixes and standardizes common patterns
 */
export function simplifyButtonName(name)
{
    if (!name) return 'Button';

    // Remove "Joystick 1 - " or "Button Button" prefixes
    name = name.replace(/^Joystick \d+ - /, '');
    name = name.replace(/^Button /, '');

    // Simplify common patterns
    name = name.replace(/Button\((\d+)\)/, 'Btn $1');
    name = name.replace(/^(\d+)$/, 'Btn $1');

    return name;
}

/**
 * Draw a connecting line between button marker and label box
 * Uses smart positioning to attach to the nearest edge of the label box
 */
export function drawConnectingLine(ctx, buttonPos, labelPos, boxHalfWidth, lineColor, isHat = false)
{
    // For hats, draw straight line to center without offset
    if (isHat)
    {
        ctx.strokeStyle = lineColor;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.moveTo(buttonPos.x, buttonPos.y);
        ctx.lineTo(labelPos.x, labelPos.y);
        ctx.stroke();
        ctx.setLineDash([]);
        return;
    }

    // For regular buttons, calculate label box dimensions using constants
    const labelWidth = ButtonFrameWidth;
    const labelHeight = ButtonFrameHeight;
    const labelX = labelPos.x - labelWidth / 2;
    const labelY = labelPos.y - labelHeight / 2;
    const labelRight = labelX + labelWidth;
    const labelLeft = labelX;
    const labelCenterY = labelPos.y;

    // Determine which edge to attach to and draw offset line
    let attachX, attachY = labelCenterY;
    let offsetX, offsetY = labelCenterY;
    const offset = 23;

    if (buttonPos.x < labelX)
    {
        // Button is to the left - attach to left edge
        attachX = labelLeft;
        offsetX = labelLeft - offset;
    } else if (buttonPos.x > labelRight)
    {
        // Button is to the right - attach to right edge
        attachX = labelRight;
        offsetX = labelRight + offset;
    } else
    {
        // Button is horizontally within box - attach to top or bottom edge
        attachX = buttonPos.x;
        if (buttonPos.y < labelY)
        {
            attachY = labelY;
            offsetY = labelY - offset;
        } else
        {
            attachY = labelY + labelHeight;
            offsetY = labelY + labelHeight + offset;
        }
        offsetX = buttonPos.x;
    }

    // Draw dashed line from button to offset point to label edge
    ctx.strokeStyle = lineColor;
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(buttonPos.x, buttonPos.y);
    ctx.lineTo(offsetX, offsetY);
    ctx.lineTo(attachX, attachY);
    ctx.stroke();
    ctx.setLineDash([]);

    // Draw circle at connection point
    ctx.fillStyle = '#aaaaaaff';
    ctx.strokeStyle = '#aaaaaaff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(attachX, attachY, 3, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
}

/**
 * Draw a single button marker (the circle on the image)
 */
export function drawButtonMarker(ctx, buttonPos, zoom, hasBinding = false, isHat = false)
{
    const handleSize = isHat ? 6 : (7 / zoom);
    // Get CSS variable colors
    const accentPrimary = getComputedStyle(document.documentElement).getPropertyValue('--accent-primary').trim();
    const bgLight = getComputedStyle(document.documentElement).getPropertyValue('--bg-light').trim();
    ctx.fillStyle = hasBinding ? accentPrimary : bgLight;
    ctx.strokeStyle = 'white';
    ctx.lineWidth = 2 / zoom;
    ctx.beginPath();
    ctx.arc(buttonPos.x, buttonPos.y, handleSize, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();
}

/**
 * Draw a binding box with button info and bindings
 * NEW IMPROVED VERSION: Accepts title and content lines array for flexible rendering
 * @param {CanvasRenderingContext2D} ctx - Canvas context
 * @param {number} x - Center X position
 * @param {number} y - Center Y position
 * @param {string} title - Title text (styled differently, shown at top)
 * @param {Array<string>} contentLines - Array of content strings to display
 * @param {boolean} compact - Use compact layout for hat directions
 * @param {Object} options - Additional options (hasBinding, buttonData, mode, onClickableBox, titleColor, contentColor, actionColor, bindingsData)
 */
export function drawButtonBox(ctx, x, y, title, contentLines = [], compact = false, options = {})
{
    const {
        hasBinding = contentLines.length > 0,
        buttonData = null,
        mode = 'normal',
        onClickableBox = null,
        titleColor = '#ccc',
        contentColor = '#ddd',
        subtleColor = '#999',
        mutedColor = '#666',
        actionColor = null,
        bindingsData = null,
        isTemplateEditor = false
    } = options;

    const width = compact ? HatFrameWidth : ButtonFrameWidth;
    const height = ButtonFrameHeight;

    const boxX = x - width / 2;
    const boxY = y - height / 2;

    // Box background
    ctx.fillStyle = hasBinding ? 'rgba(15, 18, 21, 0.95)' : 'rgba(30, 30, 30, 0.85)';
    ctx.strokeStyle = hasBinding ? '#c9c9c9ff' : '#555';
    ctx.lineWidth = 1;

    // Rounded rectangle
    roundRect(ctx, boxX, boxY, width, height, 4);
    ctx.fill();
    ctx.stroke();

    // Track clickable area if callback is provided
    // Allow clicking on buttons even if they have no bindings to view the button ID
    if (buttonData && mode === 'normal' && onClickableBox)
    {
        onClickableBox({
            x: boxX,
            y: boxY,
            width: width,
            height: height,
            buttonData: buttonData,
            bindings: bindingsData || contentLines
        });
    }

    // Render text using improved function
    RenderFrameText(ctx, x, y, width, height, title, contentLines, compact, {
        titleColor,
        contentColor,
        subtleColor,
        mutedColor,
        actionColor,
        isTemplateEditor
    });
}

/**
 * Improved text rendering for button boxes
 * Handles title, multiple content lines, truncation, and alignment
 * @param {CanvasRenderingContext2D} ctx - Canvas context
 * @param {number} x - Center X position
 * @param {number} y - Center Y position
 * @param {number} boxWidth - Box width
 * @param {number} boxHeight - Box height
 * @param {string} title - Title text at top
 * @param {Array<string>} contentLines - Array of content strings
 * @param {boolean} compact - Use compact layout
 * @param {Object} colors - Color options (titleColor, contentColor, subtleColor, mutedColor, actionColor)
 */
export function RenderFrameText(ctx, x, y, boxWidth, boxHeight, title, contentLines = [], compact = false, colors = {})
{
    const {
        titleColor = '#ccc',
        contentColor = '#ddd',
        subtleColor = '#999',
        mutedColor = '#666',
        actionColor = null,
        isTemplateEditor = false
    } = colors;

    // Calculate text layout metrics
    const padding = 4;
    const contentWidth = boxWidth - (padding * 2);
    const lineHeight = compact ? TextLineHeightCompact : TextLineHeight;
    const titleFontSize = compact ? TitleFontSizeCompact : TitleFontSize;
    const contentFontSize = compact ? ContentFontSizeCompact : ContentFontSize;
    const actionFontSize = ActionTextSize;
    const countFontSize = ActionTextSize;
    const titleSpacing = compact ? 14 : 18;

    // Helper function to truncate text to fit width
    const truncateText = (text, font, maxWidth, ellipsis = true) =>
    {
        ctx.font = font;
        if (ctx.measureText(text).width <= maxWidth)
        {
            return text;
        }

        const suffix = ellipsis ? '...' : '';
        let truncated = text;

        while (truncated.length > 0 && ctx.measureText(truncated + suffix).width > maxWidth)
        {
            truncated = truncated.slice(0, -1);
        }
        return truncated + suffix;
    };

    // Draw title at top
    ctx.fillStyle = titleColor;
    ctx.font = `${titleFontSize} "Segoe UI", sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    // Move title down 4px in template editor mode for better spacing
    const titleOffset = isTemplateEditor ? 16 : 0;
    const titleY = y - (boxHeight / 2) + (titleSpacing / 2) + padding + titleOffset;
    const truncatedTitle = truncateText(title, ctx.font, contentWidth, true);
    ctx.fillText(truncatedTitle, x, titleY);

    // Calculate content area
    const contentHeight = boxHeight - titleSpacing;
    const maxLinesAvailable = Math.floor(contentHeight / lineHeight);

    // If no content, show placeholder
    if (!contentLines || contentLines.length === 0)
    {
        ctx.fillStyle = mutedColor;
        ctx.font = `italic ${contentFontSize} "Segoe UI", sans-serif`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('(unbound)', x, y + 2);
        return;
    }

    // Determine how many lines we can show
    // For non-compact boxes, limit to NumLines; for compact, use available space
    const maxDisplayLines = compact ? maxLinesAvailable : NumLines;
    const linesToShow = Math.min(maxDisplayLines, contentLines.length);
    const showMoreIndicator = contentLines.length > linesToShow;

    // Calculate starting Y position for content (vertically center the content area)
    // Always reserve space for "+x more" indicator when at maximum display lines
    // This keeps spacing consistent for all boxes at the maximum capacity, regardless of overflow
    const isAtMaxCapacity = linesToShow === maxDisplayLines;
    const actualLines = isAtMaxCapacity ? linesToShow - 1 : linesToShow;

    // Calculate total height of all lines accounting for variable font sizes
    let totalTextHeight = 0;
    for (let i = 0; i < actualLines; i++)
    {
        const line = contentLines[i];
        const isLeftAligned = line.startsWith('[action]') || line.startsWith('[muted]');
        const fontSize = isLeftAligned ? actionFontSize : contentFontSize;
        const fontSizeNum = parseInt(fontSize);
        totalTextHeight += fontSizeNum + 3; // font size + 3px padding
    }
    // Subtract the last padding since there's no line after the last one
    if (actualLines > 0)
    {
        totalTextHeight -= 3;
    }

    const contentAreaStartY = y - (boxHeight / 2) + titleSpacing;
    const startY = contentAreaStartY + (contentHeight - totalTextHeight) / 2;

    // Render content lines
    for (let i = 0; i < actualLines; i++)
    {
        const line = contentLines[i];

        // Parse line for styling hints (e.g., "[subtle]text" or "[muted]text")
        let displayText = line;
        let color = contentColor;
        let isLeftAligned = false;

        if (line.startsWith('[action]'))
        {
            displayText = line.substring(8);
            color = actionColor || contentColor;
            isLeftAligned = true;
        } else if (line.startsWith('[subtle]'))
        {
            displayText = line.substring(8);
            color = subtleColor;
        } else if (line.startsWith('[muted]'))
        {
            displayText = line.substring(7);
            color = mutedColor;
            isLeftAligned = true;
        } else if (line.startsWith('[bright]'))
        {
            displayText = line.substring(8);
            color = contentColor;
        }

        // Use ActionTextSize for action and muted text, contentFontSize for others
        const fontSize = isLeftAligned ? actionFontSize : contentFontSize;
        ctx.fillStyle = color;
        ctx.font = `${fontSize} "Segoe UI", sans-serif`;
        ctx.textBaseline = 'middle';

        // Calculate line height based on font size (extract numeric value and add padding)
        const fontSizeNum = parseInt(fontSize);
        const lineHeightForFont = fontSizeNum + 3; // Add 3px padding between lines

        // Calculate Y position accounting for variable line heights
        let lineY = startY;
        for (let j = 0; j < i; j++)
        {
            const prevLine = contentLines[j];
            let prevIsLeftAligned = prevLine.startsWith('[action]') || prevLine.startsWith('[muted]');
            const prevFontSize = prevIsLeftAligned ? actionFontSize : contentFontSize;
            const prevFontSizeNum = parseInt(prevFontSize);
            lineY += prevFontSizeNum + 3;
        }

        // Left-align action labels and default bindings, center-align others
        const xPos = isLeftAligned ? x - contentWidth / 2 + padding : x;
        ctx.textAlign = isLeftAligned ? 'left' : 'center';

        const truncated = truncateText(displayText, ctx.font, contentWidth - (isLeftAligned ? 0 : 0), true);
        ctx.fillText(truncated, xPos, lineY);
    }

    // Show "more" indicator if needed
    if (showMoreIndicator)
    {
        const remainingCount = contentLines.length - actualLines;

        // Position at the bottom of the content area
        const bottomPadding = 3;
        const moreIndicatorY = y + (boxHeight / 2) - bottomPadding;

        ctx.fillStyle = '#aaa';
        ctx.font = `${countFontSize} "Segoe UI", sans-serif`;
        ctx.textBaseline = 'bottom';

        const countText = `+${remainingCount} more`;
        const countWidth = ctx.measureText(countText).width;
        const bgPadding = 3;
        const bgHeight = parseInt(countFontSize) + 2;

        // Draw background for count - left-aligned
        const bgX = x - contentWidth / 2 + padding;
        ctx.fillStyle = 'rgba(0, 0, 0, 0)';
        roundRect(ctx, bgX - 1, moreIndicatorY - bgHeight, countWidth + bgPadding * 2, bgHeight, 2);
        ctx.fill();

        // Draw text - left-aligned
        ctx.fillStyle = '#aaa';
        ctx.textAlign = 'left';
        ctx.fillText(countText, bgX + bgPadding, moreIndicatorY);
    }
}

/**
 * Build content lines for a button label
 * Prepares title and content array for rendering
 * @returns {Object} Object with title and contentLines array
 */
export function buildButtonLabelContent(button)
{
    const title = simplifyButtonName(button.name || 'Button');
    const contentLines = [];

    // Extract button ID
    let displayButtonId = null;
    if (button.buttonId !== undefined)
    {
        displayButtonId = button.buttonId;
    } else if (button.inputs && button.inputs.main)
    {
        const match = button.inputs.main.match(/button(\d+)/i);
        if (match)
        {
            displayButtonId = match[1];
        }
    }

    // Add button ID to content
    if (displayButtonId !== null)
    {
        contentLines.push(`[subtle]Button ${displayButtonId}`);
    }
    else if (button.inputs && button.inputs.main)
    {
        // Try to match axis in numeric format: js1_axis3 or js1_axis3_positive
        const axisMatch = button.inputs.main.match(/axis(\d+)(?:_(positive|negative))?/i);
        if (axisMatch)
        {
            const dirSymbol = axisMatch[2] === 'positive' ? '+' : axisMatch[2] === 'negative' ? '-' : '';
            const suffix = dirSymbol ? ` ${dirSymbol}` : '';
            contentLines.push(`[subtle]Axis ${axisMatch[1]}${suffix}`);
        }
        else
        {
            // Try to match Star Citizen axis format: js1_x, js1_y, js1_z, js1_rotx, js1_roty, js1_rotz, js1_slider
            const scAxisMatch = button.inputs.main.match(/_(x|y|z|rotx|roty|rotz|slider)$/i);
            if (scAxisMatch)
            {
                const axisName = scAxisMatch[1].toUpperCase();
                contentLines.push(`[subtle]Axis ${axisName}`);
            }
        }
    }
    else if (button.inputType === 'axis' && button.inputId !== undefined)
    {
        const dirSymbol = button.axisDirection === 'positive' ? '+' : (button.axisDirection === 'negative' ? '-' : '');
        const suffix = dirSymbol ? ` ${dirSymbol}` : '';
        contentLines.push(`[subtle]Axis ${button.inputId}${suffix}`);
    }

    // Check if button has a binding
    const hasBound = (button.inputs && button.inputs.main) || button.buttonId !== undefined ||
        (button.inputType && button.inputId !== undefined);
    if (!hasBound)
    {
        contentLines.push('[muted](unbound)');
    }

    return { title, contentLines };
}

/**
 * Draw a label box for a button or hat direction
 * Unified rendering for both regular buttons and hat directions
 * @param {CanvasRenderingContext2D} ctx - Canvas context
 * @param {number} x - Center X position
 * @param {number} y - Center Y position
 * @param {string} title - Label title text
 * @param {Array<string>} contentLines - Array of content lines to display
 * @param {boolean} compact - Use compact layout (for hat directions)
 * @param {number} alpha - Opacity (0-1)
 * @param {Object} colors - Optional color overrides
 */
export function DrawButtonFrame(ctx, x, y, title, contentLines, compact = false, alpha = 1, colors = {}, isTemplateEditor = false)
{
    ctx.save();
    ctx.globalAlpha = alpha;

    const width = compact ? HatFrameWidth : ButtonFrameWidth;
    const height = ButtonFrameHeight;
    const boxX = x - width / 2;
    const boxY = y - height / 2;
    const radius = 4;

    // Box background
    ctx.fillStyle = 'rgba(30, 30, 30, 0.85)';
    ctx.strokeStyle = '#555';
    ctx.lineWidth = 1.0;

    roundRect(ctx, boxX, boxY, width, height, radius);
    ctx.fill();
    ctx.stroke();

    // Use improved rendering with color defaults
    const defaultColors = {
        titleColor: '#ccc',
        contentColor: '#ddd',
        subtleColor: '#999',
        mutedColor: '#666'
    };

    RenderFrameText(ctx, x, y, width, height, title, contentLines, compact, {
        ...defaultColors,
        ...colors,
        isTemplateEditor
    });

    ctx.restore();
}

/**
 * Draw a single button label box (for template editor)
 * @deprecated Use drawLabel instead for unified rendering
 */
export function drawButtonLabel(ctx, button, title, contentLines, alpha, isTemplateEditor = false)
{
    DrawButtonFrame(ctx, button.labelPos.x, button.labelPos.y, title, contentLines, false, alpha, {}, isTemplateEditor);
}

/**
 * Draw a single button with label (for template editor)
 * Convenience function that combines label content building and rendering
 * @deprecated Use buildButtonLabelContent + drawButtonLabel instead for better separation of concerns
 */
export function drawSingleButtonLabel(ctx, button, alpha, isTemplateEditor = false)
{
    const { title, contentLines } = buildButtonLabelContent(button);
    drawButtonLabel(ctx, button, title, contentLines, alpha, isTemplateEditor);
}

/**
 * Calculate hat positions for a 4-way hat switch
 * Returns an object with positions for each direction
 */
export function getHat4WayPositions(centerX, centerY, hasPush = false)
{
    let offsetY = (HatFrameHeight / 2) + (HatSpacing * 2) + (HatSpacing / 2) + (HatFrameHeight / 2) + HatSpacing / 2;
    if (!hasPush) offsetY -= HatFrameHeight / 2 + HatSpacing;
    else offsetY += HatSpacing;

    return {
        'up': { x: centerX, y: centerY - offsetY },
        'down': { x: centerX, y: centerY + offsetY },
        'left': { x: centerX - HatFrameWidth - HatSpacing, y: centerY },
        'right': { x: centerX + HatFrameWidth + HatSpacing, y: centerY },
        'push': { x: centerX, y: centerY }
    };
}

/**
 * Calculate hat positions for a 2-way vertical hat switch (Up/Down)
 * Returns an object with positions for up and down directions
 */
export function getHat2WayVerticalPositions(centerX, centerY, hasPush = false)
{
    // For vertical: up and down boxes positioned with spacing between them
    const verticalOffset = HatFrameHeight / 2 + HatSpacing * 2;

    return {
        'up': { x: centerX, y: centerY - verticalOffset },
        'down': { x: centerX, y: centerY + verticalOffset },
        'push': { x: centerX, y: centerY + HatFrameHeight + HatSpacing * 2 }
    };
}

/**
 * Calculate hat positions for a 2-way horizontal hat switch (Left/Right)
 * Returns an object with positions for left and right directions
 */
export function getHat2WayHorizontalPositions(centerX, centerY, hasPush = false)
{
    // For horizontal: left and right boxes touching with spacing between
    // If there's a push button, it goes below at centerY
    const horizontalOffset = HatFrameWidth / 2 + HatSpacing / 2;

    return {
        'left': { x: centerX - horizontalOffset, y: centerY },
        'right': { x: centerX + horizontalOffset, y: centerY },
        'push': { x: centerX, y: centerY + HatFrameHeight + HatSpacing }
    };
}

/**
 * Get box bounds for a specific hat direction
 * Useful for hit testing and bounds checking
 */
export function getHat4WayBoxBounds(direction, centerX, centerY, hasPush = false)
{
    const positions = getHat4WayPositions(centerX, centerY, hasPush);
    if (!positions[direction])
    {
        return null;
    }

    const pos = positions[direction];
    const halfWidth = HatFrameWidth / 2;
    const halfHeight = HatFrameHeight / 2;

    return {
        x: pos.x - halfWidth,
        y: pos.y - halfHeight,
        width: HatFrameWidth,
        height: HatFrameHeight
    };
}

/**
 * Get box bounds for a specific hat direction in a 2-way vertical hat
 * Useful for hit testing and bounds checking
 */
export function getHat2WayVerticalBoxBounds(direction, centerX, centerY, hasPush = false)
{
    const positions = getHat2WayVerticalPositions(centerX, centerY, hasPush);
    if (!positions[direction])
    {
        return null;
    }

    const pos = positions[direction];
    const halfWidth = HatFrameWidth / 2;
    const halfHeight = HatFrameHeight / 2;

    return {
        x: pos.x - halfWidth,
        y: pos.y - halfHeight,
        width: HatFrameWidth,
        height: HatFrameHeight
    };
}

/**
 * Get box bounds for a specific hat direction in a 2-way horizontal hat
 * Useful for hit testing and bounds checking
 */
export function getHat2WayHorizontalBoxBounds(direction, centerX, centerY, hasPush = false)
{
    const positions = getHat2WayHorizontalPositions(centerX, centerY, hasPush);
    if (!positions[direction])
    {
        return null;
    }

    const pos = positions[direction];
    const halfWidth = HatFrameWidth / 2;
    const halfHeight = HatFrameHeight / 2;

    return {
        x: pos.x - halfWidth,
        y: pos.y - halfHeight,
        width: HatFrameWidth,
        height: HatFrameHeight
    };
}

/**
 * Unified function to draw 4-way hat boxes with customizable content and styling
 * Used by both template editor and joystick viewer for consistent rendering
 * @param {CanvasRenderingContext2D} ctx - Canvas context
 * @param {Object} hat - Hat button data
 * @param {Object} options - Rendering options
 * @param {string} options.mode - 'template' or 'viewer' mode
 * @param {number} options.alpha - Opacity (0-1)
 * @param {Function} options.getContentForDirection - Function(direction, input) that returns array of content lines
 * @param {Object} options.colors - Color overrides (titleColor, contentColor, subtleColor, mutedColor, actionColor)
 * @param {Function} options.onClickableBox - Callback for registering clickable boxes
 * @param {Object} options.buttonDataForDirection - Function(direction) that returns buttonData for clickable tracking
 */
export function drawHat4WayBoxes(ctx, hat, options = {})
{
    const {
        mode = 'template',
        alpha = 1,
        getContentForDirection = null,
        colors = {},
        onClickableBox = null,
        buttonDataForDirection = null,
        bindingsByDirection = null,
        isTemplateEditor = false
    } = options;

    ctx.save();
    ctx.globalAlpha = alpha;

    // Check if push button exists
    const hasPush = hat.inputs && hat.inputs['push'];

    // Calculate all positions using centralized helper
    const positions = getHat4WayPositions(hat.labelPos.x, hat.labelPos.y, hasPush);

    // Calculate spacing for hat name position
    // When there's a push button, the up direction is pushed down further, so we need more spacing above
    let offsetY = HatFrameHeight * 2;
    if (!hasPush) offsetY -= HatFrameHeight / 2 + HatSpacing;

    const titleGap = -8;
    const titleY = hat.labelPos.y - offsetY - titleGap;

    // Draw hat name above
    ctx.fillStyle = colors.titleColor || '#aaa';
    ctx.font = `${HatTitleFontSize} "Segoe UI", sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const simplifiedName = simplifyButtonName(hat.name || 'Hat');
    ctx.fillText(simplifiedName, hat.labelPos.x, titleY);

    // Draw each direction box
    const directions = ['up', 'down', 'left', 'right', 'push'];
    directions.forEach(dir =>
    {
        // Only draw if this direction has inputs
        if (!hat.inputs || !hat.inputs[dir])
        {
            return;
        }

        const pos = positions[dir];
        const input = hat.inputs[dir];

        // Get content lines for this direction
        let contentLines = [];
        if (getContentForDirection)
        {
            contentLines = getContentForDirection(dir, input);
        }

        // Determine label
        const label = dir === 'push' ? 'Push' : dir.charAt(0).toUpperCase();

        // For template mode, use unified label drawing
        if (mode === 'template')
        {
            DrawButtonFrame(ctx, pos.x, pos.y, label, contentLines, true, alpha, colors, isTemplateEditor);
        }
        else
        {
            // Get actual bindings data if available
            const actualBindings = bindingsByDirection ? bindingsByDirection[dir] : null;

            // Use drawButtonBox for viewer mode with full functionality
            const boxOptions = {
                hasBinding: contentLines.length > 0,
                buttonData: buttonDataForDirection ? buttonDataForDirection(dir) : null,
                mode: mode,
                onClickableBox: onClickableBox,
                titleColor: colors.titleColor || '#ccc',
                contentColor: colors.contentColor || '#ddd',
                subtleColor: colors.subtleColor || '#999',
                mutedColor: colors.mutedColor || '#666',
                actionColor: colors.actionColor || null,
                bindingsData: actualBindings || contentLines
            };

            drawButtonBox(ctx, pos.x, pos.y, label, contentLines, true, boxOptions);
        }
    });

    ctx.restore();
}

/**
 * Draw hat switch labels in a plus pattern (for template editor)
 * Uses improved text rendering system
 */
export function drawHat4WayFrames(ctx, button, alpha, handleSize, zoom)
{
    // Use unified rendering function with template editor styling
    drawHat4WayBoxes(ctx, button, {
        mode: 'template',
        alpha: alpha,
        isTemplateEditor: true,
        getContentForDirection: (dir, input) =>
        {
            const contentLines = [];
            // Show input if available
            const match = input.match(/button(\d+)/i);
            if (match)
            {
                contentLines.push(`[subtle]Button ${match[1]}`);
            }
            return contentLines;
        },
        colors: {
            titleColor: '#aaa',
            contentColor: '#ddd',
            subtleColor: '#999',
            mutedColor: '#666'
        }
    });
}

/**
 * Unified function to draw 2-way vertical hat boxes (Up/Down)
 * @param {CanvasRenderingContext2D} ctx - Canvas context
 * @param {Object} hat - Hat button data
 * @param {Object} options - Rendering options
 */
export function drawHat2WayVerticalBoxes(ctx, hat, options = {})
{
    const {
        mode = 'template',
        alpha = 1,
        getContentForDirection = null,
        colors = {},
        onClickableBox = null,
        buttonDataForDirection = null,
        bindingsByDirection = null,
        isTemplateEditor = false
    } = options;

    ctx.save();
    ctx.globalAlpha = alpha;

    const hasPush = hat.inputs && hat.inputs['push'];
    const positions = getHat2WayVerticalPositions(hat.labelPos.x, hat.labelPos.y, hasPush);

    let offsetY = HatFrameHeight * 2;
    if (!hasPush) offsetY -= HatFrameHeight / 2 + HatSpacing;

    const titleGap = -8;
    const titleY = hat.labelPos.y - offsetY - titleGap;

    // Draw hat name above
    ctx.fillStyle = colors.titleColor || '#aaa';
    ctx.font = `${HatTitleFontSize} "Segoe UI", sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const simplifiedName = simplifyButtonName(hat.name || 'Hat');
    ctx.fillText(simplifiedName, hat.labelPos.x, titleY);

    // Draw each direction box
    const directions = ['up', 'down', 'push'];
    directions.forEach(dir =>
    {
        // Skip push button if it doesn't have a binding
        if (dir === 'push' && (!hat.inputs || !hat.inputs['push']))
        {
            return;
        }

        const pos = positions[dir];
        const input = hat.inputs && hat.inputs[dir] ? hat.inputs[dir] : null;

        let contentLines = [];
        if (getContentForDirection && input)
        {
            contentLines = getContentForDirection(dir, input);
        }

        const label = dir === 'push' ? 'Push' : dir.charAt(0).toUpperCase();

        if (mode === 'template')
        {
            DrawButtonFrame(ctx, pos.x, pos.y, label, contentLines, true, alpha, colors, isTemplateEditor);
        }
        else
        {
            const actualBindings = bindingsByDirection ? bindingsByDirection[dir] : null;
            const boxOptions = {
                hasBinding: contentLines.length > 0,
                buttonData: buttonDataForDirection ? buttonDataForDirection(dir) : null,
                mode: mode,
                onClickableBox: onClickableBox,
                titleColor: colors.titleColor || '#ccc',
                contentColor: colors.contentColor || '#ddd',
                subtleColor: colors.subtleColor || '#999',
                mutedColor: colors.mutedColor || '#666',
                actionColor: colors.actionColor || null,
                bindingsData: actualBindings || contentLines
            };
            drawButtonBox(ctx, pos.x, pos.y, label, contentLines, true, boxOptions);
        }
    });

    ctx.restore();
}

/**
 * Unified function to draw 2-way horizontal hat boxes (Left/Right)
 * @param {CanvasRenderingContext2D} ctx - Canvas context
 * @param {Object} hat - Hat button data
 * @param {Object} options - Rendering options
 */
export function drawHat2WayHorizontalBoxes(ctx, hat, options = {})
{
    const {
        mode = 'template',
        alpha = 1,
        getContentForDirection = null,
        colors = {},
        onClickableBox = null,
        buttonDataForDirection = null,
        bindingsByDirection = null,
        isTemplateEditor = false
    } = options;

    ctx.save();
    ctx.globalAlpha = alpha;

    const hasPush = hat.inputs && hat.inputs['push'];
    const positions = getHat2WayHorizontalPositions(hat.labelPos.x, hat.labelPos.y, hasPush);

    const titleGap = -8;
    const titleY = hat.labelPos.y - HatFrameHeight - HatSpacing - titleGap;

    // Draw hat name above
    ctx.fillStyle = colors.titleColor || '#aaa';
    ctx.font = `${HatTitleFontSize} "Segoe UI", sans-serif`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    const simplifiedName = simplifyButtonName(hat.name || 'Hat');
    ctx.fillText(simplifiedName, hat.labelPos.x, titleY);

    // Draw each direction box
    const directions = ['left', 'right', 'push'];
    directions.forEach(dir =>
    {
        // Skip push button if it doesn't have a binding
        if (dir === 'push' && (!hat.inputs || !hat.inputs['push']))
        {
            return;
        }

        const pos = positions[dir];
        const input = hat.inputs && hat.inputs[dir] ? hat.inputs[dir] : null;

        let contentLines = [];
        if (getContentForDirection && input)
        {
            contentLines = getContentForDirection(dir, input);
        }

        const label = dir === 'push' ? 'Push' : (dir === 'left' ? '◄' : '►');

        if (mode === 'template')
        {
            DrawButtonFrame(ctx, pos.x, pos.y, label, contentLines, true, alpha, colors, isTemplateEditor);
        }
        else
        {
            const actualBindings = bindingsByDirection ? bindingsByDirection[dir] : null;
            const boxOptions = {
                hasBinding: contentLines.length > 0,
                buttonData: buttonDataForDirection ? buttonDataForDirection(dir) : null,
                mode: mode,
                onClickableBox: onClickableBox,
                titleColor: colors.titleColor || '#ccc',
                contentColor: colors.contentColor || '#ddd',
                subtleColor: colors.subtleColor || '#999',
                mutedColor: colors.mutedColor || '#666',
                actionColor: colors.actionColor || null,
                bindingsData: actualBindings || contentLines
            };
            drawButtonBox(ctx, pos.x, pos.y, label, contentLines, true, boxOptions);
        }
    });

    ctx.restore();
}
