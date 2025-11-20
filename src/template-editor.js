// Safely access TAURI APIs
let invoke, open, save;

if (window.__TAURI__)
{
    invoke = window.__TAURI__.core.invoke;
    ({ open, save } = window.__TAURI__.dialog);
}

// Import shared rendering utilities
import
{
    ButtonFrameWidth,
    ButtonFrameHeight,
    roundRect,
    simplifyButtonName,
    drawConnectingLine,
    drawButtonMarker,
    drawSingleButtonLabel,
    drawHat4WayFrames,
    drawButtonBox,
    RenderFrameText,
    getHat4WayPositions,
    getHat4WayBoxBounds,
    HatFrameWidth,
    HatFrameHeight
} from './button-renderer.js';
import { initializeTemplatePagesUI, refreshTemplatePagesUI } from './template-editor-v2.js';

// Lazy imports - will be loaded when needed
let parseInputDisplayName, parseInputShortName, getInputType, toStarCitizenFormat;

// Load utilities when template editor initializes
async function loadUtilities()
{
    if (!parseInputDisplayName)
    {
        const utils = await import('./input-utils.js');
        parseInputDisplayName = utils.parseInputDisplayName;
        parseInputShortName = utils.parseInputShortName;
        getInputType = utils.getInputType;
        toStarCitizenFormat = utils.toStarCitizenFormat;
    }
}

// State
let templateData = {
    name: '',
    joystickModel: '',
    joystickNumber: 2, // Default to joystick 2 (for dual stick setups) - deprecated, use per-stick joystickNumber
    leftStick: { joystickNumber: 1, buttons: [] }, // Left stick config - deprecated, use pages instead
    rightStick: { joystickNumber: 2, buttons: [] }, // Right stick config - deprecated, use pages instead
    version: '1.0',
    pages: []
};

let currentStick = 'right'; // Currently editing 'left' or 'right'
let currentPageId = null;
let canvas, ctx;
let loadedImage = null;
let zoom = 1.0;
let pan = { x: 0, y: 0 };

// Camera positions for each stick (persisted separately)
let leftStickCamera = { zoom: 1.0, pan: { x: 0, y: 0 } };
let rightStickCamera = { zoom: 1.0, pan: { x: 0, y: 0 } };
let selectedButtonId = null;
let mode = 'view'; // 'view', 'placing-button', 'placing-label'
let tempButton = null;
let originalButton = null; // Store original button data for cancel functionality
let draggingHandle = null;
let isPanning = false;
let lastPanPosition = { x: 0, y: 0 };

// Snapping grid for better alignment when dragging boxes
const SNAP_GRID = 10; // pixels

function generatePageId()
{
    if (window.crypto && window.crypto.randomUUID)
    {
        return window.crypto.randomUUID();
    }
    return `page_${Date.now()}_${Math.floor(Math.random() * 1000)}`;
}

function syncLegacyStickReferences()
{
    if (!Array.isArray(templateData.pages))
    {
        templateData.pages = [];
    }

    templateData.leftStick = templateData.pages[0] || { joystickNumber: 1, buttons: [] };
    templateData.rightStick = templateData.pages[1] || { joystickNumber: 2, buttons: [] };
}

function ensureTemplatePages()
{
    if (!Array.isArray(templateData.pages))
    {
        templateData.pages = [];
    }

    if (templateData.pages.length === 0)
    {
        const leftPage = templateData.leftStick || { joystickNumber: 1, buttons: [] };
        if (!leftPage.id)
        {
            leftPage.id = generatePageId();
        }
        leftPage.name = leftPage.name || 'Left Stick';
        templateData.pages.push(leftPage);

        const rightPage = templateData.rightStick || { joystickNumber: 2, buttons: [] };
        if (!rightPage.id)
        {
            rightPage.id = generatePageId();
        }
        rightPage.name = rightPage.name || 'Right Stick';
        templateData.pages.push(rightPage);
    }
    else
    {
        templateData.pages.forEach((page, index) =>
        {
            if (!page.id)
            {
                page.id = generatePageId();
            }
            if (!Array.isArray(page.buttons))
            {
                page.buttons = [];
            }
            if (page.joystickNumber === undefined)
            {
                page.joystickNumber = index === 0 ? 1 : 2;
            }
            if (!page.name)
            {
                page.name = index === 0 ? 'Left Stick' : 'Right Stick';
            }
        });
    }

    syncLegacyStickReferences();

    if (!currentPageId && templateData.pages.length)
    {
        currentPageId = templateData.pages[0].id;
    }
}

function handleTemplatePagesChanged()
{
    syncLegacyStickReferences();
    markAsChanged();
    updateButtonList();
    redraw();
}

function handleTemplatePageSelected(pageId)
{
    if (!pageId)
    {
        return;
    }

    currentPageId = pageId;

    // Find the page and load its data
    if (Array.isArray(templateData.pages))
    {
        const page = templateData.pages.find(p => p.id === pageId);
        if (page)
        {
            // Update button list to show this page's buttons
            updateButtonList();

            // Clear selection when switching pages
            selectButton(null);

            // Load the page's image (or mirrored image) - this will call redraw when done
            loadPageImage(page);
        }
    }
}// Helper function to resize image to max width of 1024px while maintaining aspect ratio
function resizeImage(img, maxWidth = 1024, callback)
{
    // If image is already smaller than maxWidth, use it as is
    if (img.width <= maxWidth)
    {
        if (callback)
        {
            // Use setTimeout to make it async like the resize case
            setTimeout(() => callback(img), 0);
        }
        return;
    }

    // Calculate new dimensions maintaining aspect ratio
    const ratio = maxWidth / img.width;
    const newWidth = maxWidth;
    const newHeight = Math.round(img.height * ratio);

    // Create a canvas to resize the image
    const resizeCanvas = document.createElement('canvas');
    resizeCanvas.width = newWidth;
    resizeCanvas.height = newHeight;

    const resizeCtx = resizeCanvas.getContext('2d');
    resizeCtx.imageSmoothingEnabled = true;
    resizeCtx.imageSmoothingQuality = 'high';

    // Draw the resized image
    resizeCtx.drawImage(img, 0, 0, newWidth, newHeight);

    // Create a new image from the resized canvas
    const resizedImg = new Image();
    resizedImg.onload = () =>
    {
        if (callback)
        {
            callback(resizedImg);
        }
    };
    resizedImg.src = resizeCanvas.toDataURL('image/png');
}

// Load image for a specific page (handles mirroring)
function loadPageImage(page)
{
    if (!page) return;

    const processImage = (imageDataUrl) =>
    {
        const img = new Image();

        const handleImageLoad = () =>
        {
            resizeImage(img, 1024, (resizedImg) =>
            {
                loadedImage = resizedImg;
                redraw();
            });
        };

        // Handle both cached and uncached images
        img.onload = handleImageLoad;
        img.src = imageDataUrl;

        // For cached images, check after a microtask
        setTimeout(() =>
        {
            if (img.complete && img.naturalWidth > 0)
            {
                handleImageLoad();
            }
        }, 0);
    };

    // Check if this page mirrors another page
    if (page.mirror_from_page_id)
    {
        const mirrorPage = templateData.pages.find(p => p.id === page.mirror_from_page_id);
        if (mirrorPage && mirrorPage.image_data_url)
        {
            processImage(mirrorPage.image_data_url);
            return;
        }
    }

    // Use this page's own image
    if (page.image_data_url)
    {
        processImage(page.image_data_url);
    }
    else
    {
        // No image for this page
        loadedImage = null;
        redraw();
    }
}

// Joystick input detection
let detectingInput = false;
let inputDetectionTimeout = null; // Track timeout to clear it when restarting
let hatDetectionTimeout = null; // Track hat detection timeout to clear it when restarting
let currentDetectionSessionId = null; // Track current detection session to prevent race conditions
let currentHatDetectionSessionId = null; // Track current hat detection session

// Track unsaved changes
let hasUnsavedChanges = false;

// Track current template file path for auto-saving
let currentTemplateFilePath = null;

// Export initialization function for tab system
window.initializeTemplateEditor = function ()
{
    if (canvas) return; // Already initialized

    // Load utilities first
    loadUtilities();

    ensureTemplatePages();

    canvas = document.getElementById('editor-canvas');
    ctx = canvas.getContext('2d');

    // Ensure stick structures are initialized properly
    if (!templateData.leftStick || typeof templateData.leftStick !== 'object')
    {
        templateData.leftStick = { joystickNumber: 1, buttons: [] };
    }
    if (!templateData.leftStick.buttons || !Array.isArray(templateData.leftStick.buttons))
    {
        templateData.leftStick.buttons = [];
    }

    if (!templateData.rightStick || typeof templateData.rightStick !== 'object')
    {
        templateData.rightStick = { joystickNumber: 2, buttons: [] };
    }
    if (!templateData.rightStick.buttons || !Array.isArray(templateData.rightStick.buttons))
    {
        templateData.rightStick.buttons = [];
    }

    initializeEventListeners();
    loadPersistedTemplate();

    initializeTemplatePagesUI({
        template: templateData,
        getTemplate: () => templateData,
        onPagesChanged: handleTemplatePagesChanged,
        onPageSelected: handleTemplatePageSelected
    });

    // Ensure canvas is sized after layout is complete
    requestAnimationFrame(() =>
    {
        resizeCanvas();
    });

    window.addEventListener('resize', resizeCanvas);
};

function initializeEventListeners()
{
    // Page selector buttons are now handled dynamically by template-editor-v2.js
    // No need to listen on a static dropdown anymore

    document.getElementById('save-template-btn').addEventListener('click', saveTemplate);
    document.getElementById('save-template-as-btn').addEventListener('click', saveTemplateAs);
    document.getElementById('load-template-btn').addEventListener('click', loadTemplate);

    // Sidebar controls
    document.getElementById('template-name').addEventListener('input', (e) =>
    {
        templateData.name = e.target.value;
        markAsChanged();
        if (window.updateTemplateIndicator)
        {
            const savedFileName = localStorage.getItem('templateFileName');
            window.updateTemplateIndicator(e.target.value, savedFileName);
        }
    });

    document.getElementById('joystick-model').addEventListener('input', (e) =>
    {
        templateData.joystickModel = e.target.value;
        markAsChanged();
    });

    // Legacy image controls removed - per-page images now handled in template page modal
    document.getElementById('new-template-btn').addEventListener('click', newTemplate);
    document.getElementById('add-button-btn').addEventListener('click', startAddButton);
    document.getElementById('delete-button-btn').addEventListener('click', deleteSelectedButton);
    document.getElementById('clear-all-btn').addEventListener('click', clearAllButtons);
    document.getElementById('mirror-template-btn').addEventListener('click', mirrorTemplate);
    document.getElementById('change-joystick-number-btn').addEventListener('click', changeAllJoystickNumbers);



    // Zoom controls
    document.getElementById('zoom-in-btn').addEventListener('click', () => zoomBy(0.1));
    document.getElementById('zoom-out-btn').addEventListener('click', () => zoomBy(-0.1));
    document.getElementById('zoom-fit-btn').addEventListener('click', fitToScreen);
    document.getElementById('zoom-reset-btn').addEventListener('click', resetZoom);    // Canvas events
    canvas.addEventListener('mousedown', onCanvasMouseDown);
    canvas.addEventListener('dblclick', onCanvasDoubleClick);
    canvas.addEventListener('mousemove', onCanvasMouseMove);
    canvas.addEventListener('wheel', onCanvasWheel, { passive: false });
    canvas.addEventListener('contextmenu', (e) => e.preventDefault()); // Prevent right-click menu

    // Global mouseup to catch releases outside canvas (fixes panning stuck bug)
    document.addEventListener('mouseup', onCanvasMouseUp);

    // Keyboard shortcuts
    document.addEventListener('keydown', (e) =>
    {
        // Don't trigger shortcuts when modals are open
        const buttonModal = document.getElementById('button-modal');
        const pageModal = document.getElementById('page-modal');
        if ((buttonModal && buttonModal.style.display === 'flex') || (pageModal && pageModal.style.display === 'flex'))
        {
            return; // Modal is open, don't handle shortcuts
        }

        if (e.key.toLowerCase() === 'f' && loadedImage)
        {
            fitToScreen();
        }

        // Tab key to navigate pages
        if (e.key === 'Tab')
        {
            e.preventDefault(); // Prevent default tab focus behavior
            navigatePages(e.shiftKey ? -1 : 1); // Shift+Tab goes back, Tab goes forward
        }
    });

    // Modal
    document.getElementById('button-modal-cancel').addEventListener('click', closeButtonModal);
    document.getElementById('button-modal-save').addEventListener('click', saveButtonDetails);
    document.getElementById('button-modal-delete').addEventListener('click', deleteCurrentButton);
    document.getElementById('button-modal-detect').addEventListener('click', startInputDetection);
    document.getElementById('button-type-select').addEventListener('change', onButtonTypeChange);

    // Hat detection buttons
    document.querySelectorAll('.hat-detect-btn').forEach(btn =>
    {
        btn.addEventListener('click', (e) =>
        {
            const direction = e.target.dataset.direction;
            startHatInputDetection(direction);
        });
    });

    // Hat clear buttons
    document.querySelectorAll('.hat-clear-btn').forEach(btn =>
    {
        btn.addEventListener('click', (e) =>
        {
            const direction = e.target.dataset.direction;
            clearHatDirection(direction);
        });
    });

    // Simple button clear button
    document.getElementById('button-modal-clear').addEventListener('click', clearSimpleButtonInput);

    // Hidden file inputs
    // Legacy image file input - removed since per-page images are now handled in page modal
    // Keep the element for backward compatibility if needed
}

function navigatePages(direction)
{
    if (!templateData.pages || templateData.pages.length === 0) return;

    const currentIndex = templateData.pages.findIndex(p => p.id === currentPageId);
    if (currentIndex === -1) return;

    const nextIndex = (currentIndex + direction + templateData.pages.length) % templateData.pages.length;
    const nextPageId = templateData.pages[nextIndex].id;

    selectPageInternal(nextPageId);
}

function selectPageInternal(pageId)
{
    if (!pageId || !Array.isArray(templateData.pages)) return;

    currentPageId = pageId;

    // Find the page and load its data
    const page = templateData.pages.find(p => p.id === pageId);
    if (page)
    {
        // Update button list to show this page's buttons
        updateButtonList();

        // Clear selection when switching pages
        selectButton(null);

        // Load the page's image (or mirrored image) - this will call redraw when done
        loadPageImage(page);
    }

    // Notify template-editor-v2 about the page change - use window.selectPage if available
    if (window.selectPage)
    {
        window.selectPage(pageId);
    }
    else if (window.templateEditorCallbacks?.onPageSelected)
    {
        window.templateEditorCallbacks.onPageSelected(pageId);
    }
}

function resizeCanvas()
{
    const container = document.getElementById('canvas-container');
    const rect = container.getBoundingClientRect();

    // Set CSS size for display (doesn't affect internal resolution)
    canvas.style.width = rect.width + 'px';
    canvas.style.height = rect.height + 'px';

    // Set internal resolution to match CSS size (with device pixel ratio for crisp rendering)
    const dpr = window.devicePixelRatio || 1;
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;

    // Note: We'll apply DPR scaling in redraw() to avoid accumulation

    console.log('Canvas resized:', rect.width, 'x', rect.height, '(DPR:', dpr + ')');

    redraw();
}

// Stick switching
function switchStick(stick, skipRedraw = false)
{
    if (currentStick === stick) return;

    // Save current camera position before switching
    if (currentStick === 'left')
    {
        leftStickCamera = { zoom, pan: { x: pan.x, y: pan.y } };
        saveCameraPosition(); // Persist to localStorage
    }
    else
    {
        rightStickCamera = { zoom, pan: { x: pan.x, y: pan.y } };
        saveCameraPosition(); // Persist to localStorage
    }

    currentStick = stick;

    console.log('Switching to stick:', stick);
    console.log('Left stick buttons:', templateData.leftStick);
    console.log('Right stick buttons:', templateData.rightStick);

    // Note: stick selector buttons removed - now using page selector dropdown
    // This function is kept for backward compatibility but no longer updates UI buttons

    // Restore saved camera position for this stick
    if (stick === 'left')
    {
        zoom = leftStickCamera.zoom;
        pan = { x: leftStickCamera.pan.x, y: leftStickCamera.pan.y };
    }
    else
    {
        zoom = rightStickCamera.zoom;
        pan = { x: rightStickCamera.pan.x, y: rightStickCamera.pan.y };
    }
    updateZoomDisplay();

    // Clear selection
    selectButton(null);

    // Update button list and redraw (unless told to skip redraw)
    updateButtonList();
    if (!skipRedraw)
    {
        redraw();
    }
}

// Get current stick's button array
function getCurrentButtons()
{
    // If using TemplateV2 pages and a page is selected, use that
    if (currentPageId && Array.isArray(templateData.pages))
    {
        const page = templateData.pages.find(p => p.id === currentPageId);
        if (page)
        {
            if (!Array.isArray(page.buttons))
            {
                page.buttons = [];
            }
            return page.buttons;
        }
    }

    // Fallback to legacy stick-based logic
    if (currentStick === 'left')
    {
        // Handle nested structure: { joystickNumber: 1, buttons: [...] }
        if (templateData.leftStick && typeof templateData.leftStick === 'object' && !Array.isArray(templateData.leftStick))
        {
            if (!Array.isArray(templateData.leftStick.buttons))
            {
                templateData.leftStick.buttons = [];
            }
            return templateData.leftStick.buttons;
        }
        // Handle flat array structure: [...]
        if (!Array.isArray(templateData.leftStick))
        {
            templateData.leftStick = [];
        }
        return templateData.leftStick;
    }
    else
    {
        // Handle nested structure: { joystickNumber: 2, buttons: [...] }
        if (templateData.rightStick && typeof templateData.rightStick === 'object' && !Array.isArray(templateData.rightStick))
        {
            if (!Array.isArray(templateData.rightStick.buttons))
            {
                templateData.rightStick.buttons = [];
            }
            return templateData.rightStick.buttons;
        }
        // Handle flat array structure: [...]
        if (!Array.isArray(templateData.rightStick))
        {
            templateData.rightStick = [];
        }
        return templateData.rightStick;
    }
}

function getCurrentStickData()
{
    // If using TemplateV2 pages and a page is selected, return that page
    if (currentPageId && Array.isArray(templateData.pages))
    {
        const page = templateData.pages.find(p => p.id === currentPageId);
        if (page)
        {
            return page;
        }
    }

    // Fallback to legacy stick data
    return currentStick === 'left' ? templateData.leftStick : templateData.rightStick;
}

// Get current page (for TemplateV2)
function getCurrentPage()
{
    if (currentPageId && Array.isArray(templateData.pages))
    {
        return templateData.pages.find(p => p.id === currentPageId);
    }
    return null;
}

function getCurrentStickJoystickNumber()
{
    const stickData = getCurrentStickData();

    // For pages, use the joystickNumber field (same as stick data)
    if (stickData && stickData.joystickNumber)
    {
        return stickData.joystickNumber;
    }

    // Fallback to global joystick number
    if (templateData.joystickNumber)
    {
        return templateData.joystickNumber;
    }

    // Final fallback based on current stick
    return currentStick === 'left' ? 1 : 2;
}

function getInputDisplayInfo(button, jsNumOverride = null)
{
    const info = {
        shortLabel: null,
        fullId: null,
        type: null
    };

    if (!button)
    {
        return info;
    }

    const jsNum = jsNumOverride || getCurrentStickJoystickNumber();

    // Get current page prefix
    const currentPage = getCurrentPage();
    const pagePrefix = currentPage ? currentPage.joystick_prefix : null;

    const normalizePrefix = (inputString) =>
    {
        if (!inputString)
        {
            return null;
        }

        const lower = inputString.toLowerCase();

        // If we have a page prefix, use it
        if (pagePrefix)
        {
            if (lower.match(/^(js|gp)\d+_/))
            {
                return lower.replace(/^(js|gp)\d+_/, `${pagePrefix}_`);
            }
        }

        if (lower.match(/^(js|gp)\d+_/))
        {
            return lower.replace(/^(js|gp)\d+_/, `js${jsNum}_`);
        }
        return lower;
    };

    const setFromString = (inputString) =>
    {
        const normalized = normalizePrefix(inputString);
        if (!normalized)
        {
            return;
        }

        info.fullId = normalized;

        if (normalized.includes('_axis'))
        {
            info.type = 'axis';
            const axisMatch = normalized.match(/axis(\d+)(?:_(positive|negative))?/);
            if (axisMatch)
            {
                let dirSymbol = '';
                if (axisMatch[2] === 'positive')
                {
                    dirSymbol = '+';
                }
                else if (axisMatch[2] === 'negative')
                {
                    dirSymbol = '-';
                }
                else if (axisMatch[2])
                {
                    dirSymbol = axisMatch[2];
                }

                info.shortLabel = dirSymbol ? `Axis ${axisMatch[1]} ${dirSymbol}` : `Axis ${axisMatch[1]}`;
            }
            else
            {
                info.shortLabel = 'Axis';
            }
        }
        else if (normalized.match(/^(js|gp)\d+_(x|y|z|rotx|roty|rotz|slider)$/))
        {
            // Star Citizen axis names (e.g., js1_x, js1_y, js1_rotx)
            info.type = 'axis';
            const scAxisMatch = normalized.match(/_(x|y|z|rotx|roty|rotz|slider)$/);
            if (scAxisMatch)
            {
                const axisName = scAxisMatch[1].toUpperCase();
                info.shortLabel = `Axis ${axisName}`;
            }
            else
            {
                info.shortLabel = 'Axis';
            }
        }
        else if (normalized.includes('_button'))
        {
            info.type = 'button';
            const btnMatch = normalized.match(/button(\d+)/);
            info.shortLabel = btnMatch ? `Button ${btnMatch[1]}` : 'Button';
        }
        else if (normalized.includes('_hat'))
        {
            info.type = 'hat';
            const hatMatch = normalized.match(/hat(\d+)_(up|down|left|right)/);
            if (hatMatch)
            {
                const hatNum = hatMatch[1];
                const direction = hatMatch[2].charAt(0).toUpperCase() + hatMatch[2].slice(1);
                info.shortLabel = `Hat ${hatNum} ${direction}`;
            }
            else
            {
                info.shortLabel = 'Hat Switch';
            }
        }
        else
        {
            info.type = 'input';
            info.shortLabel = normalized;
        }
    };

    if (button.inputs && button.inputs.main)
    {
        if (typeof button.inputs.main === 'string')
        {
            setFromString(button.inputs.main);
        }
        else if (typeof button.inputs.main === 'object')
        {
            const main = button.inputs.main;

            if (main.type === 'axis' && main.id !== undefined)
            {
                const directionSuffix = main.direction ? `_${main.direction}` : '';
                setFromString(`js${jsNum}_axis${main.id}${directionSuffix}`);
            }
            else if (main.type === 'button' && main.id !== undefined)
            {
                setFromString(`js${jsNum}_button${main.id}`);
            }
            else if (typeof main.input === 'string')
            {
                setFromString(main.input);
            }
            else if (main.id !== undefined)
            {
                info.shortLabel = `Input ${main.id}`;
                info.fullId = main.id.toString();
            }
        }
    }
    else if (button.buttonId !== undefined && button.buttonId !== null)
    {
        setFromString(`js${jsNum}_button${button.buttonId}`);
    }
    else if (button.inputType && button.inputId !== undefined)
    {
        if (button.inputType === 'axis')
        {
            const directionSuffix = button.axisDirection ? `_${button.axisDirection}` : '';
            setFromString(`js${jsNum}_axis${button.inputId}${directionSuffix}`);
        }
        else if (button.inputType === 'button')
        {
            setFromString(`js${jsNum}_button${button.inputId}`);
        }
        else if (button.inputType === 'hat')
        {
            const hatDirection = button.hatDirection || 'up';
            setFromString(`js${jsNum}_hat${button.inputId}_${hatDirection}`);
        }
    }

    return info;
}

function updateSimpleInputPreview(button = null)
{
    const displayInfo = getInputDisplayInfo(button);
    const numberEl = document.getElementById('button-id-display');
    const idEl = document.getElementById('button-full-id-display');

    if (!numberEl || !idEl)
    {
        return;
    }

    if (displayInfo.shortLabel)
    {
        numberEl.textContent = displayInfo.shortLabel;
        idEl.textContent = displayInfo.fullId || displayInfo.shortLabel;
    }
    else
    {
        numberEl.textContent = '—';
        idEl.textContent = '—';
    }
}

// Set current stick's button array
function setCurrentButtons(buttons)
{
    // If using TemplateV2 pages and a page is selected, use that
    if (currentPageId && Array.isArray(templateData.pages))
    {
        const page = templateData.pages.find(p => p.id === currentPageId);
        if (page)
        {
            page.buttons = buttons;
            syncLegacyStickReferences();
            return;
        }
    }

    // Fallback to legacy stick-based logic
    if (currentStick === 'left')
    {
        // Handle nested structure
        if (templateData.leftStick && typeof templateData.leftStick === 'object' && !Array.isArray(templateData.leftStick))
        {
            templateData.leftStick.buttons = buttons;
        }
        else
        {
            templateData.leftStick = buttons;
        }
    }
    else
    {
        // Handle nested structure
        if (templateData.rightStick && typeof templateData.rightStick === 'object' && !Array.isArray(templateData.rightStick))
        {
            templateData.rightStick.buttons = buttons;
        }
        else
        {
            templateData.rightStick = buttons;
        }
    }
}

// New template
async function newTemplate()
{
    if ((templateData.name ||
        templateData.pages.length > 0 ||
        templateData.leftStick.buttons.length > 0 ||
        templateData.rightStick.buttons.length > 0))
    {
        const showConfirmation = window.showConfirmation;
        if (!showConfirmation)
        {
            console.error('showConfirmation not available');
            return;
        }

        const confirmed = await showConfirmation(
            'Start a new template? Any unsaved changes will be lost.',
            'New Template',
            'Start New',
            'Cancel'
        );

        if (!confirmed)
        {
            return;
        }
    }

    // Reset all data
    templateData = {
        name: '',
        joystickModel: '',
        joystickNumber: 2,
        leftStick: { joystickNumber: 1, buttons: [] },
        rightStick: { joystickNumber: 2, buttons: [] },
        version: '1.0',
        pages: []
    };

    ensureTemplatePages();
    refreshTemplatePagesUI(templateData);

    // Reset UI
    document.getElementById('template-name').value = '';
    document.getElementById('joystick-model').value = '';

    // Reset canvas
    loadedImage = null;
    currentStick = 'right';
    selectedButtonId = null;
    zoom = 1.0;
    pan = { x: 0, y: 0 };

    // Reset camera positions for both sticks
    leftStickCamera = { zoom: 1.0, pan: { x: 0, y: 0 } };
    rightStickCamera = { zoom: 1.0, pan: { x: 0, y: 0 } };

    // Update UI
    switchStick('right');
    resizeCanvas();

    // Clear localStorage
    localStorage.removeItem('currentTemplate');
    localStorage.removeItem('templateFileName');
    localStorage.removeItem('templateFilePath');
    localStorage.removeItem('leftStickCamera');
    localStorage.removeItem('rightStickCamera');
    hasUnsavedChanges = false;
    currentTemplateFilePath = null; // Clear the file path for new template
    updateUnsavedIndicator();

    // Reset header template name
    if (window.updateTemplateIndicator)
    {
        window.updateTemplateIndicator('Untitled Template');
    }
}

// Handle image type selection
// Legacy image loading functions removed - per-page images now handled in template page modal

// Drawing functions
function redraw()
{
    if (!ctx) return;

    // Get canvas display size
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;

    // Clear canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // Determine if image should be flipped (page-based mirroring only)
    let shouldFlip = false;
    const currentPage = getCurrentPage();
    if (currentPage)
    {
        // Check if this page mirrors another page (flip required)
        shouldFlip = !!currentPage.mirror_from_page_id;
    }

    ctx.save();

    // Apply DPR scaling first (to work with physical pixels)
    ctx.scale(dpr, dpr);

    // Apply zoom and pan (in logical pixels)
    ctx.translate(pan.x, pan.y);
    ctx.scale(zoom, zoom);

    // Draw image if available
    if (loadedImage)
    {
        // Enable smooth image rendering
        ctx.imageSmoothingEnabled = true;
        ctx.imageSmoothingQuality = 'high';

        // Draw image with optional flip based on mirroring settings
        ctx.save();

        if (shouldFlip)
        {
            ctx.translate(loadedImage.width, 0);
            ctx.scale(-1, 1);
        }
        ctx.drawImage(loadedImage, 0, 0);
        ctx.restore();
    }

    // Draw all buttons for current stick (without flip)
    // This works even if there's no background image
    const buttons = getCurrentButtons();
    if (Array.isArray(buttons))
    {
        buttons.forEach(button =>
        {
            drawButton(button);
        });
    }

    // Draw temp button while placing
    if (tempButton)
    {
        drawButton(tempButton, true);
    }

    ctx.restore();
}

// Expose redraw globally for use by template-editor-v2.js
window.redraw = redraw;

// Expose function to update loadedImage from external modules
window.setLoadedImage = function (img)
{
    loadedImage = img;
};

function drawButton(button, isTemp = false)
{
    const alpha = isTemp ? 0.7 : 1.0;
    const isHat = button.buttonType === 'hat4way';

    // Draw line connecting button to label
    if (button.labelPos)
    {
        ctx.save();
        ctx.globalAlpha = alpha;

        let lineColor = '#d9534f'; // Default to bound color

        if (isHat)
        {
            // For hats, check if at least the four cardinal directions are bound
            const hasCardinalDirections = button.inputs &&
                button.inputs.up &&
                button.inputs.down &&
                button.inputs.left &&
                button.inputs.right;

            lineColor = hasCardinalDirections ? '#d9534f' : '#666'; // Bound color if all cardinals exist, grey otherwise
        }

        // Use shared drawConnectingLine function
        // Note: Need to scale offset for zoom level in template editor
        const labelWidth = isHat ? 0 : 140;
        drawConnectingLine(ctx, button.buttonPos, button.labelPos, labelWidth / 2, lineColor, isHat);
        ctx.restore();
    }

    // Draw button position marker using shared function
    ctx.save();
    ctx.globalAlpha = alpha;
    drawButtonMarker(ctx, button.buttonPos, zoom, !isHat, isHat);
    ctx.restore();

    // Draw label box(es) using shared functions
    if (button.labelPos)
    {
        if (isHat)
        {
            drawHat4WayFrames(ctx, button, alpha, (7 / zoom), zoom);
        }
        else
        {
            drawSingleButtonLabel(ctx, button, alpha, (7 / zoom), zoom);
        }
    }

    // Highlight if selected
    if (button.id === selectedButtonId && !isTemp)
    {
        ctx.save();
        ctx.strokeStyle = '#e67e72';
        ctx.lineWidth = 3;

        // Highlight the connecting line with brighter color
        if (button.labelPos)
        {
            drawConnectingLine(ctx, button.buttonPos, button.labelPos, isHat ? 0 : ButtonFrameWidth / 2, '#9ae764ff', isHat);
        }

        // Highlight the label box border
        if (button.labelPos)
        {
            const isHat = button.buttonType === 'hat4way';

            if (isHat)
            {
                // For hats, highlight the center push box
                const boxWidth = HatFrameWidth;
                const boxHeight = HatFrameHeight;
                const x = button.labelPos.x - boxWidth / 2;
                const y = button.labelPos.y - boxHeight / 2;

                roundRect(ctx, x, y, boxWidth, boxHeight, 4);
                ctx.stroke();
            }
            else
            {
                // For simple buttons, highlight the label box
                const labelWidth = ButtonFrameWidth;
                const labelHeight = ButtonFrameHeight;
                const x = button.labelPos.x - labelWidth / 2;
                const y = button.labelPos.y - labelHeight / 2;

                roundRect(ctx, x, y, labelWidth, labelHeight, 4);
                ctx.stroke();
            }
        }

        ctx.restore();
    }
}

// Note: drawSingleButtonLabel and drawHat4WayLabels are now imported from button-renderer.js
// Note: roundRect and simplifyButtonName are now imported from button-renderer.js

// Canvas interaction
function getCanvasCoords(event)
{
    const rect = canvas.getBoundingClientRect();
    const x = (event.clientX - rect.left - pan.x) / zoom;
    const y = (event.clientY - rect.top - pan.y) / zoom;
    return { x, y };
}

function onCanvasMouseDown(event)
{
    const coords = getCanvasCoords(event);

    // Middle click or right click for panning
    if (event.button === 1 || event.button === 2)
    {
        isPanning = true;
        lastPanPosition = { x: event.clientX, y: event.clientY };
        canvas.style.cursor = 'grabbing';
        event.preventDefault();
        return;
    }

    // Only handle left click for button operations below this point
    if (event.button !== 0) return;

    if (mode === 'view')
    {
        // Can select buttons even without an image for reference
        // Check if clicking on a handle
        const handle = findHandleAtPosition(coords);
        if (handle)
        {
            draggingHandle = handle;
            selectButton(handle.buttonId);
            return;
        }

        // Check if clicking on a button
        const button = findButtonAtPosition(coords);
        if (button)
        {
            selectButton(button.id);
        } else
        {
            selectButton(null);
        }
    } else if (mode === 'placing-button')
    {
        // Place the button position
        tempButton = {
            id: Date.now(),
            name: '',
            buttonPos: { ...coords },
            labelPos: null
        };
        mode = 'placing-label';
        redraw();
    } else if (mode === 'placing-label')
    {
        // Place the label position
        tempButton.labelPos = { ...coords };
        mode = 'view';
        redraw();

        // Open modal to get button name
        openButtonModal(tempButton);
    }
}

// Snap coordinate to grid
function snapToGrid(value, gridSize = SNAP_GRID)
{
    return Math.round(value / gridSize) * gridSize;
}

function onCanvasMouseMove(event)
{
    if (isPanning)
    {
        const deltaX = event.clientX - lastPanPosition.x;
        const deltaY = event.clientY - lastPanPosition.y;

        pan.x += deltaX;
        pan.y += deltaY;

        lastPanPosition = { x: event.clientX, y: event.clientY };
        redraw();
        return;
    }

    if (draggingHandle)
    {
        const coords = getCanvasCoords(event);
        const snappedCoords = {
            x: snapToGrid(coords.x),
            y: snapToGrid(coords.y)
        };
        const buttons = getCurrentButtons();
        const button = buttons.find(b => b.id === draggingHandle.buttonId);

        if (button)
        {
            if (draggingHandle.type === 'button')
            {
                button.buttonPos = { ...snappedCoords };
            } else if (draggingHandle.type === 'label')
            {
                button.labelPos = { ...snappedCoords };
            }
            markAsChanged();
            redraw();
        }
    }

    // Update cursor
    if (mode === 'placing-button' || mode === 'placing-label')
    {
        canvas.style.cursor = 'crosshair';
    } else if (draggingHandle)
    {
        canvas.style.cursor = 'move';
    } else
    {
        const coords = getCanvasCoords(event);
        const handle = findHandleAtPosition(coords);
        canvas.style.cursor = handle ? 'move' : 'default';
    }
}

function onCanvasMouseUp(event)
{
    if (isPanning)
    {
        isPanning = false;
        canvas.style.cursor = 'default';
        // Save camera position after panning
        saveCameraPosition();
    }

    draggingHandle = null;
}

// Helper function to save current camera position
function saveCameraPosition()
{
    if (currentStick === 'left')
    {
        leftStickCamera = { zoom, pan: { x: pan.x, y: pan.y } };
        localStorage.setItem('leftStickCamera', JSON.stringify(leftStickCamera));
    }
    else
    {
        rightStickCamera = { zoom, pan: { x: pan.x, y: pan.y } };
        localStorage.setItem('rightStickCamera', JSON.stringify(rightStickCamera));
    }
}

function onCanvasWheel(event)
{
    event.preventDefault();

    const delta = -event.deltaY / 1000;
    zoomBy(delta, event);
}

function onCanvasDoubleClick(event)
{
    // Allow editing even if no image is loaded, as long as we are in view mode
    if (mode !== 'view') return;

    const coords = getCanvasCoords(event);
    console.log('Double click at', coords);

    // Check if double-clicking on a button
    const button = findButtonAtPosition(coords);
    if (button)
    {
        console.log('Found button', button.id);
        editButtonFromList(button.id);
    }
    else
    {
        console.log('No button found at position');
    }
}

function findHandleAtPosition(pos)
{
    const handleSize = 12 / zoom; // For button position markers
    const buttons = getCurrentButtons();

    for (const button of buttons)
    {
        // Check button position handle (keep the red dot)
        const distButton = Math.sqrt(
            Math.pow(pos.x - button.buttonPos.x, 2) +
            Math.pow(pos.y - button.buttonPos.y, 2)
        );
        if (distButton <= handleSize)
        {
            return { buttonId: button.id, type: 'button' };
        }

        // Check if clicking on label box area
        if (button.labelPos)
        {
            const isHat = button.buttonType === 'hat4way';

            if (isHat)
            {
                // Use centralized hat position calculation
                const hasPush = button.inputs && button.inputs['push'];
                const directions = ['up', 'down', 'left', 'right', 'push'];

                for (const dir of directions)
                {
                    // Only check directions that have inputs
                    if (!button.inputs || !button.inputs[dir])
                    {
                        continue;
                    }

                    const bounds = getHat4WayBoxBounds(dir, button.labelPos.x, button.labelPos.y, hasPush);
                    if (bounds &&
                        pos.x >= bounds.x && pos.x <= bounds.x + bounds.width &&
                        pos.y >= bounds.y && pos.y <= bounds.y + bounds.height)
                    {
                        return { buttonId: button.id, type: 'label' };
                    }
                }
            }
            else
            {
                // For simple buttons, check the single label box
                // Use world coordinates (don't divide by zoom)
                const labelWidth = ButtonFrameWidth;
                const labelHeight = ButtonFrameHeight;
                const x = button.labelPos.x - labelWidth / 2;
                const y = button.labelPos.y - labelHeight / 2;

                if (pos.x >= x && pos.x <= x + labelWidth &&
                    pos.y >= y && pos.y <= y + labelHeight)
                {
                    return { buttonId: button.id, type: 'label' };
                }
            }
        }
    }

    return null;
}

function findButtonAtPosition(pos)
{
    const handleSize = 12 / zoom;
    const buttons = getCurrentButtons();

    for (const button of buttons)
    {
        // Check if clicking near button position
        const dist = Math.sqrt(
            Math.pow(pos.x - button.buttonPos.x, 2) +
            Math.pow(pos.y - button.buttonPos.y, 2)
        );
        if (dist <= handleSize)
        {
            return button;
        }

        // Check if clicking on label box
        if (button.labelPos)
        {
            if (button.buttonType === 'hat4way')
            {
                const hasPush = button.inputs && button.inputs['push'];
                const directions = ['up', 'down', 'left', 'right', 'push'];

                for (const dir of directions)
                {
                    // Only check directions that have inputs
                    if (!button.inputs || !button.inputs[dir])
                    {
                        continue;
                    }

                    const bounds = getHat4WayBoxBounds(dir, button.labelPos.x, button.labelPos.y, hasPush);
                    if (bounds &&
                        pos.x >= bounds.x && pos.x <= bounds.x + bounds.width &&
                        pos.y >= bounds.y && pos.y <= bounds.y + bounds.height)
                    {
                        return button;
                    }
                }
            }
            else
            {
                const labelWidth = ButtonFrameWidth;
                const labelHeight = ButtonFrameHeight;
                const x = button.labelPos.x - labelWidth / 2;
                const y = button.labelPos.y - labelHeight / 2;

                if (pos.x >= x && pos.x <= x + labelWidth &&
                    pos.y >= y && pos.y <= y + labelHeight)
                {
                    return button;
                }
            }
        }
    }

    return null;
}

// Zoom functions
function zoomBy(delta, event = null)
{
    const oldZoom = zoom;
    zoom = Math.max(0.1, Math.min(5, zoom + delta));

    if (event)
    {
        // Zoom towards mouse position
        const rect = canvas.getBoundingClientRect();
        const mouseX = event.clientX - rect.left;
        const mouseY = event.clientY - rect.top;

        pan.x = mouseX - (mouseX - pan.x) * (zoom / oldZoom);
        pan.y = mouseY - (mouseY - pan.y) * (zoom / oldZoom);
    }

    updateZoomDisplay();
    saveCameraPosition();
    redraw();
}

function resetZoom()
{
    if (!loadedImage) return;

    // Reset to 100% zoom
    zoom = 1.0;

    // Center image in canvas at actual size
    const scaledWidth = loadedImage.width * zoom;
    const scaledHeight = loadedImage.height * zoom;
    pan.x = (canvas.width - scaledWidth) / 2;
    pan.y = (canvas.height - scaledHeight) / 2;

    updateZoomDisplay();
    saveCameraPosition();
    redraw();
}

function fitToScreen()
{
    if (!loadedImage) return;

    // Fit image to canvas with padding
    const padding = 80; // More generous padding for better visibility
    const availableWidth = canvas.width - (padding * 2);
    const availableHeight = canvas.height - (padding * 2);

    const scaleX = availableWidth / loadedImage.width;
    const scaleY = availableHeight / loadedImage.height;
    zoom = Math.min(scaleX, scaleY);

    // Clamp zoom to reasonable bounds
    zoom = Math.max(0.1, Math.min(5, zoom));

    // Center image in viewport
    const scaledWidth = loadedImage.width * zoom;
    const scaledHeight = loadedImage.height * zoom;
    pan.x = (canvas.width - scaledWidth) / 2;
    pan.y = (canvas.height - scaledHeight) / 2;

    updateZoomDisplay();
    saveCameraPosition();
    redraw();
}

function updateZoomDisplay()
{
    document.getElementById('zoom-level').textContent = `${Math.round(zoom * 100)}%`;
}

// Button management
async function startAddButton()
{
    // Get the current page and its device configuration
    const currentPage = getCurrentPage();
    if (!currentPage)
    {
        const showAlert = window.showAlert || alert;
        await showAlert('Please select or create a page first before adding buttons.', 'No Page Selected');
        return;
    }

    // For new pages-based system: check if device is configured
    if (!currentPage.device_uuid)
    {
        const showAlert = window.showAlert || alert;
        await showAlert(
            `Please configure a device for the "${currentPage.name || 'Untitled Page'}" page first.\n\n` +
            `Click the "Edit" button on the page, then:\n` +
            `1. Click "🎯 Press Button on Device"\n` +
            `2. Press any button on your joystick\n` +
            `3. Click "Save Page"\n\n` +
            `After that, you'll be able to add buttons to this page.`,
            'Configure Device First'
        );
        return;
    }

    mode = 'placing-button';
    canvas.style.cursor = 'crosshair';
    selectButton(null);
}

function highlightLoadImageButton()
{
    // Legacy function - image loading now happens in page modal
    // This function is kept for compatibility but does nothing
    return;
}


function selectButton(buttonId)
{
    selectedButtonId = buttonId;

    // Update button list UI
    document.querySelectorAll('.button-item').forEach(item =>
    {
        if (parseInt(item.dataset.buttonId) === buttonId)
        {
            item.classList.add('selected');
        } else
        {
            item.classList.remove('selected');
        }
    });

    // Enable/disable delete button
    document.getElementById('delete-button-btn').disabled = (buttonId === null);

    redraw();
}

async function deleteSelectedButton(event)
{
    if (event)
    {
        event.preventDefault();
        event.stopPropagation();
    }

    if (selectedButtonId === null) return;

    // Find the button to get its name for the confirmation message
    const buttons = getCurrentButtons();
    const buttonToDelete = buttons.find(b => b.id === selectedButtonId);
    const buttonName = buttonToDelete ? buttonToDelete.name : 'Button';

    const showConfirmation = window.showConfirmation;
    if (!showConfirmation)
    {
        console.error('showConfirmation not available');
        return;
    }

    const confirmDelete = await showConfirmation(
        `Delete button "${buttonName}"?`,
        'Delete Button',
        'Delete',
        'Cancel'
    );

    if (!confirmDelete)
    {
        // User cancelled the deletion - do nothing
        return;
    }

    // Proceed with deletion
    const updatedButtons = buttons.filter(b => b.id !== selectedButtonId);
    setCurrentButtons(updatedButtons);
    selectedButtonId = null;

    markAsChanged();
    updateButtonList();
    redraw();
}

async function clearAllButtons()
{
    const buttons = getCurrentButtons();
    if (buttons.length === 0) return;

    // Get current page name for better messaging
    let pageName = 'current page';
    const currentPage = getCurrentPage();
    if (currentPage && currentPage.name)
    {
        pageName = `"${currentPage.name}"`;
    }
    else if (!currentPage)
    {
        // Legacy mode - use stick name
        pageName = `${currentStick} stick`;
    }

    const showConfirmation = window.showConfirmation;
    if (!showConfirmation)
    {
        console.error('showConfirmation not available');
        return;
    }

    const confirmed = await showConfirmation(
        `Are you sure you want to clear all ${buttons.length} button(s) from ${pageName}? This cannot be undone.`,
        'Clear All Buttons',
        'Clear All',
        'Cancel',
        'btn-danger'
    );

    if (!confirmed) return;

    setCurrentButtons([]);
    selectedButtonId = null;
    markAsChanged();
    updateButtonList();
    redraw();
}

async function mirrorTemplate()
{
    // Check if we have pages to mirror
    if (!templateData.pages || templateData.pages.length < 2)
    {
        await window.showAlert('You need at least 2 pages to use the mirror feature.', 'Not Enough Pages');
        return;
    }

    // Show mirror modal
    const modal = document.getElementById('mirror-template-modal');
    const sourceSelect = document.getElementById('mirror-source-page-select');
    const destSelect = document.getElementById('mirror-dest-page-select');
    const confirmBtn = document.getElementById('mirror-template-confirm-btn');
    const cancelBtn = document.getElementById('mirror-template-cancel-btn');

    // Populate dropdowns with pages
    sourceSelect.innerHTML = '<option value="">-- Select a page --</option>';
    destSelect.innerHTML = '<option value="">-- Select a page --</option>';

    templateData.pages.forEach(page =>
    {
        const option1 = document.createElement('option');
        option1.value = page.id;
        option1.textContent = page.name || `Page ${page.id}`;
        sourceSelect.appendChild(option1);

        const option2 = document.createElement('option');
        option2.value = page.id;
        option2.textContent = page.name || `Page ${page.id}`;
        destSelect.appendChild(option2);
    });

    // Auto-select the current page in the source dropdown
    if (currentPageId)
    {
        sourceSelect.value = currentPageId;
    }

    // Show modal
    modal.style.display = 'flex';

    // Wait for user action
    const result = await new Promise(resolve =>
    {
        const handleConfirm = () =>
        {
            const sourcePageId = sourceSelect.value;
            const destPageId = destSelect.value;

            if (!sourcePageId || !destPageId)
            {
                window.showAlert('Please select both a source and destination page.', 'Selection Required');
                return;
            }

            if (sourcePageId === destPageId)
            {
                window.showAlert('Source and destination pages must be different.', 'Invalid Selection');
                return;
            }

            cleanup();
            resolve({ sourcePageId, destPageId });
        };

        const handleCancel = () =>
        {
            cleanup();
            resolve(null);
        };

        const cleanup = () =>
        {
            confirmBtn.removeEventListener('click', handleConfirm);
            cancelBtn.removeEventListener('click', handleCancel);
            modal.style.display = 'none';
        };

        confirmBtn.addEventListener('click', handleConfirm);
        cancelBtn.addEventListener('click', handleCancel);
    });

    if (!result) return;

    // Find source and destination pages
    const sourcePage = templateData.pages.find(p => p.id === result.sourcePageId);
    const destPage = templateData.pages.find(p => p.id === result.destPageId);

    if (!sourcePage || !destPage)
    {
        await window.showAlert('Could not find selected pages.', 'Error');
        return;
    }

    // Get the image width for mirroring
    let imageWidth = null;

    // Try to get image width from source page
    if (sourcePage.image_data_url)
    {
        const img = new Image();
        img.src = sourcePage.image_data_url;
        await new Promise(resolve => { img.onload = resolve; });
        imageWidth = img.width;
    }
    else if (sourcePage.mirror_from_page_id)
    {
        // Source page mirrors another page, get that page's image
        const mirrorPage = templateData.pages.find(p => p.id === sourcePage.mirror_from_page_id);
        if (mirrorPage && mirrorPage.image_data_url)
        {
            const img = new Image();
            img.src = mirrorPage.image_data_url;
            await new Promise(resolve => { img.onload = resolve; });
            imageWidth = img.width;
        }
    }

    if (!imageWidth)
    {
        await window.showAlert('Source page does not have an image loaded.', 'No Image');
        return;
    }

    // Mirror all button positions from source to destination
    const sourceButtons = sourcePage.buttons || [];

    if (sourceButtons.length === 0)
    {
        await window.showAlert('Source page has no buttons to mirror.', 'No Buttons');
        return;
    }

    // Create mirrored copies of all buttons
    const mirroredButtons = sourceButtons.map(button =>
    {
        const mirroredButton = JSON.parse(JSON.stringify(button)); // Deep copy
        mirroredButton.id = Date.now() + Math.random(); // Give new unique ID

        // Mirror button position
        mirroredButton.buttonPos.x = imageWidth - mirroredButton.buttonPos.x;

        // Mirror label position
        if (mirroredButton.labelPos)
        {
            mirroredButton.labelPos.x = imageWidth - mirroredButton.labelPos.x;
        }

        return mirroredButton;
    });

    // Replace destination page buttons with mirrored copies
    destPage.buttons = mirroredButtons;

    markAsChanged();
    syncLegacyStickReferences();

    // Switch to destination page to show the mirrored result
    if (window.handleTemplatePageSelected)
    {
        window.handleTemplatePageSelected(destPage.id);
    }

    await window.showAlert(
        `Successfully mirrored ${sourceButtons.length} button(s) from "${sourcePage.name}" to "${destPage.name}"!`,
        'Mirror Complete'
    );
}


async function changeAllJoystickNumbers()
{
    // Get current stick data
    const currentStickData = currentStick === 'left' ? templateData.leftStick : templateData.rightStick;
    const buttons = currentStickData.buttons;

    if (buttons.length === 0)
    {
        await window.showAlert('No buttons in current view to update', 'No Buttons');
        return;
    }

    // Get current joystick number and toggle it
    const currentJsNum = currentStickData.joystickNumber || 1;
    const targetJsNum = currentJsNum === 1 ? 2 : 1;

    const confirmed = await window.showConfirmation(
        `Change all button joystick numbers in ${currentStick} stick from js${currentJsNum} to js${targetJsNum}?`,
        'Change Joystick Numbers',
        'Change',
        'Cancel'
    );

    if (!confirmed)
    {
        return;
    }

    // Update the stick's joystick number
    currentStickData.joystickNumber = targetJsNum;

    // Update all button inputs in the current stick
    buttons.forEach(button =>
    {
        if (button.inputs)
        {
            Object.keys(button.inputs).forEach(key =>
            {
                const input = button.inputs[key];
                if (typeof input === 'string')
                {
                    // Replace the old js number with the new one
                    button.inputs[key] = input.replace(/^js[1-2]_/, `js${targetJsNum}_`);
                }
            });
        }
    });

    markAsChanged();
    updateButtonList();
    redraw();

    await window.showAlert(
        `All buttons in ${currentStick} stick updated to use joystick ${targetJsNum}!`,
        'Update Complete'
    );
}

function updateButtonList()
{
    const listEl = document.getElementById('button-list');
    const buttons = getCurrentButtons();

    if (buttons.length === 0)
    {
        listEl.innerHTML = '<div class="empty-state-small">No buttons added yet</div>';
        document.getElementById('delete-button-btn').disabled = true;
        return;
    }

    let html = '';
    buttons.forEach(button =>
    {
        let inputInfo = '';

        if (button.buttonType === 'hat4way' && button.inputs)
        {
            const directions = [];
            if (button.inputs.up) directions.push('↑');
            if (button.inputs.down) directions.push('↓');
            if (button.inputs.left) directions.push('←');
            if (button.inputs.right) directions.push('→');
            if (button.inputs.push) directions.push('⬇');
            inputInfo = ` - Hat (${directions.join(' ')})`;
        }
        else
        {
            const displayInfo = getInputDisplayInfo(button);
            if (displayInfo.shortLabel)
            {
                inputInfo = ` - ${displayInfo.shortLabel}`;
            }
            else if (button.inputType && button.inputId !== undefined)
            {
                const label = button.inputType === 'axis' ? 'Axis' : (button.inputType === 'button' ? 'Button' : 'Input');
                inputInfo = ` - ${label} ${button.inputId}`;
            }
        }

        html += `
      <div class="button-item ${button.id === selectedButtonId ? 'selected' : ''}" 
           data-button-id="${button.id}"
           onclick="selectButtonFromList(${button.id})"
           ondblclick="editButtonFromList(${button.id})">
        <div class="button-item-name">${button.name || 'Unnamed Button'}${inputInfo}</div>
        <div class="button-item-coords">
          Button: (${Math.round(button.buttonPos.x)}, ${Math.round(button.buttonPos.y)})
          ${button.labelPos ? `<br>Label: (${Math.round(button.labelPos.x)}, ${Math.round(button.labelPos.y)})` : ''}
        </div>
      </div>
    `;
    });

    listEl.innerHTML = html;
}

window.selectButtonFromList = function (buttonId)
{
    selectButton(buttonId);
};

window.editButtonFromList = function (buttonId)
{
    const buttons = getCurrentButtons();
    const button = buttons.find(b => b.id === buttonId);
    if (!button) return;

    // Store original button data for cancel functionality
    originalButton = button;
    // Create a deep copy for editing
    tempButton = JSON.parse(JSON.stringify(button));

    // Open modal with current values
    document.getElementById('button-modal').style.display = 'flex';
    document.getElementById('button-name-input').value = button.name || '';

    // Set button type
    const buttonType = button.buttonType || 'simple';
    document.getElementById('button-type-select').value = buttonType;
    onButtonTypeChange(); // Update UI sections

    // Load buttonId for simple buttons
    if (buttonType === 'simple')
    {
        updateSimpleInputPreview(tempButton);
    }
    // If it's a hat, populate the detected inputs
    else if (buttonType === 'hat4way' && button.inputs)
    {
        updateHatDetectionButtons(button.inputs);
        // Get joystick number for full ID display
        const currentStickData = currentStick === 'left' ? templateData.leftStick : templateData.rightStick;
        const jsNum = (currentStickData && currentStickData.joystickNumber) || templateData.joystickNumber || 1;

        // Display hat direction IDs with full ID strings
        if (button.inputs.up && button.inputs.up.id !== undefined)
        {
            document.querySelector('[data-direction="up"].hat-id-display').textContent = `${button.inputs.up.id} (js${jsNum}_button${button.inputs.up.id})`;
        }
        if (button.inputs.down && button.inputs.down.id !== undefined)
        {
            document.querySelector('[data-direction="down"].hat-id-display').textContent = `${button.inputs.down.id} (js${jsNum}_button${button.inputs.down.id})`;
        }
        if (button.inputs.left && button.inputs.left.id !== undefined)
        {
            document.querySelector('[data-direction="left"].hat-id-display').textContent = `${button.inputs.left.id} (js${jsNum}_button${button.inputs.left.id})`;
        }
        if (button.inputs.right && button.inputs.right.id !== undefined)
        {
            document.querySelector('[data-direction="right"].hat-id-display').textContent = `${button.inputs.right.id} (js${jsNum}_button${button.inputs.right.id})`;
        }
        if (button.inputs.push && button.inputs.push.id !== undefined)
        {
            document.querySelector('[data-direction="push"].hat-id-display').textContent = `${button.inputs.push.id} (js${jsNum}_button${button.inputs.push.id})`;
        }
    }
    else
    {
        resetHatDetectionButtons();
    }

    document.getElementById('button-name-input').focus();

    // Allow Enter to save
    const input = document.getElementById('button-name-input');
    const enterHandler = (e) =>
    {
        if (e.key === 'Enter')
        {
            saveButtonDetails();
            input.removeEventListener('keypress', enterHandler);
        }
    };
    input.addEventListener('keypress', enterHandler);
};

// Button modal
function openButtonModal(button)
{
    document.getElementById('button-modal').style.display = 'flex';
    document.getElementById('button-name-input').value = button.name || '';

    // Default to simple button type
    document.getElementById('button-type-select').value = 'simple';
    onButtonTypeChange();

    // Clear buttonId display for new buttons
    document.getElementById('button-id-display').textContent = '—';
    document.getElementById('button-full-id-display').textContent = '—';

    // Reset hat detection buttons and displays
    resetHatDetectionButtons();
    document.querySelectorAll('.hat-id-display').forEach(display => display.textContent = '—');

    // Clear any pending timeouts from previous detection session
    if (inputDetectionTimeout !== null)
    {
        clearTimeout(inputDetectionTimeout);
        inputDetectionTimeout = null;
    }

    if (hatDetectionTimeout !== null)
    {
        clearTimeout(hatDetectionTimeout);
        hatDetectionTimeout = null;
    }

    // Reset input detection status display
    document.getElementById('input-detection-status').style.display = 'none';
    document.getElementById('input-detection-status').textContent = '';
    document.getElementById('input-detection-status').style.color = '';

    // Reset hat detection status display
    document.getElementById('hat-detection-status').style.display = 'none';
    document.getElementById('hat-detection-status').textContent = '';
    document.getElementById('hat-detection-status').style.color = '';

    // Reset detectingInput flag
    detectingInput = false;

    document.getElementById('button-name-input').focus();

    // Allow Enter to save
    const input = document.getElementById('button-name-input');
    const enterHandler = (e) =>
    {
        if (e.key === 'Enter')
        {
            saveButtonDetails();
            input.removeEventListener('keypress', enterHandler);
        }
    };
    input.addEventListener('keypress', enterHandler);
}

function closeButtonModal()
{
    // Stop any active input detection
    if (detectingInput)
    {
        stopInputDetection();
    }

    document.getElementById('button-modal').style.display = 'none';

    // Only cancel if this was a new button being placed
    if (tempButton && mode === 'placing-label')
    {
        tempButton = null;
        mode = 'view';
        redraw();
    }

    // Clear references (changes are discarded if user canceled)
    tempButton = null;
    originalButton = null;
}

async function saveButtonDetails()
{
    const name = document.getElementById('button-name-input').value.trim();

    if (!name)
    {
        const showAlert = window.showAlert || alert;
        await showAlert('Please enter a button name', 'Missing Name');
        return;
    }

    if (tempButton)
    {
        tempButton.name = name;

        const buttonType = document.getElementById('button-type-select').value;
        tempButton.buttonType = buttonType;

        // Save buttonId for simple buttons
        if (buttonType === 'simple')
        {
            // For simple buttons, keep existing buttonId if it was already set
            // It's display-only now, set only through auto-detection
            if (!tempButton.inputs)
            {
                tempButton.inputs = {};
            }
        }
        // Save hat direction IDs
        else if (buttonType === 'hat4way')
        {
            if (!tempButton.inputs)
            {
                tempButton.inputs = {};
            }

            // Hat IDs are set only through auto-detection, so we just preserve what was already set
            // The inputs object is already populated by the detection process
        }

        // Check if this is a new button or editing an existing one
        const buttons = getCurrentButtons();
        const existingIndex = buttons.findIndex(b => b.id === tempButton.id);
        if (existingIndex === -1)
        {
            // New button - add to current stick
            buttons.push(tempButton);
            setCurrentButtons(buttons);
        }
        else
        {
            // Editing existing button - update the button in the array directly
            buttons[existingIndex] = tempButton;
            setCurrentButtons(buttons);
        }

        markAsChanged();
        selectButton(tempButton.id);
        tempButton = null;
        originalButton = null;
    }

    updateButtonList();
    redraw();
    closeButtonModal();
}

// Delete button from modal
async function deleteCurrentButton(event)
{
    if (event)
    {
        event.preventDefault();
        event.stopPropagation();
    }

    if (!tempButton)
    {
        console.warn('deleteCurrentButton called but tempButton is null');
        return;
    }

    // Check if button still exists BEFORE showing confirmation
    const buttonsBeforeConfirm = getCurrentButtons();
    const indexBeforeConfirm = buttonsBeforeConfirm.findIndex(b => b.id === tempButton.id);

    if (indexBeforeConfirm === -1)
    {
        const showAlert = window.showAlert || alert;
        await showAlert('Error: This button has already been deleted!', 'Delete Button');
        closeButtonModal();
        tempButton = null;
        updateButtonList();
        redraw();
        return;
    }

    // Import showConfirmation from main.js (available globally via window)
    const showConfirmation = window.showConfirmation;
    if (!showConfirmation)
    {
        console.error('showConfirmation not available');
        return;
    }

    const confirmDelete = await showConfirmation(
        `Delete button "${tempButton.name}"?`,
        'Delete Button',
        'Delete',
        'Cancel'
    );

    if (!confirmDelete)
    {
        // User cancelled the deletion - do nothing and keep modal open
        return;
    }


    // Proceed with deletion - verify button still exists (check again in case something changed)
    const buttons = getCurrentButtons();
    const index = buttons.findIndex(b => b.id === tempButton.id);

    if (index !== -1)
    {
        buttons.splice(index, 1);
        setCurrentButtons(buttons);
        markAsChanged();
        console.log('Button deleted successfully');
    }
    else
    {
        console.warn('Button was deleted between confirmation and deletion!');
    }

    // Clear references and close modal
    selectButton(null);
    tempButton = null;
    updateButtonList();
    redraw();
    closeButtonModal();
}

// Button type change handler
function onButtonTypeChange()
{
    const buttonType = document.getElementById('button-type-select').value;

    // Show/hide appropriate input sections
    if (buttonType === 'simple')
    {
        document.getElementById('simple-input-section').style.display = 'block';
        document.getElementById('hat-input-section').style.display = 'none';
    }
    else if (buttonType === 'hat4way')
    {
        document.getElementById('simple-input-section').style.display = 'none';
        document.getElementById('hat-input-section').style.display = 'block';

        // Initialize inputs object if needed
        if (tempButton && !tempButton.inputs)
        {
            tempButton.inputs = {};
        }
    }

    // Update tempButton type
    if (tempButton)
    {
        tempButton.buttonType = buttonType;
    }
}

// Helper function to detect input with dual-stage trigger support
// When a second button is detected before the first releases, use the second one
async function detectInputWithDualStageSupport(sessionId, timeoutSecs = 10)
{
    const result = await invoke('wait_for_input_binding', {
        timeoutSecs: timeoutSecs,
        sessionId: sessionId.toString()
    });

    if (!result)
    {
        return null; // Timeout or no input
    }

    // Store the first detected input
    const firstInput = result;
    console.log('[DUAL-STAGE] First input detected:', firstInput.input_string);

    // Wait briefly to see if a second input comes in before the first releases
    // This is useful for dual-stage triggers
    const dualStageWaitTime = 300; // milliseconds to wait for second input
    const secondResult = await Promise.race([
        invoke('wait_for_input_binding', {
            timeoutSecs: Math.ceil(dualStageWaitTime / 1000),
            sessionId: (sessionId + '-secondary').toString()
        }),
        new Promise(resolve => setTimeout(() => resolve(null), dualStageWaitTime))
    ]);

    if (secondResult && secondResult.input_string && secondResult.input_string !== firstInput.input_string)
    {
        console.log('[DUAL-STAGE] Second input detected while first held:', secondResult.input_string, '- using second input');
        return secondResult; // Use the second input instead
    }

    console.log('[DUAL-STAGE] No secondary input detected, using first:', firstInput.input_string);
    return firstInput; // Use the first input
}

// Hat switch input detection
async function startHatInputDetection(direction)
{
    if (detectingInput)
    {
        return;
    }

    detectingInput = true;
    const btn = document.querySelector(`[data-direction="${direction}"]`);
    const originalText = btn.textContent;
    btn.textContent = 'Detecting...';
    btn.disabled = true;

    document.getElementById('hat-detection-status').textContent = `Press ${direction}...`;
    document.getElementById('hat-detection-status').style.display = 'block';
    document.getElementById('hat-detection-status').style.color = '';

    // Generate unique session ID for this detection
    const thisSessionId = Date.now() + Math.random();
    currentHatDetectionSessionId = thisSessionId;
    console.log('[HAT-DETECTION] Starting hat detection session:', thisSessionId, 'for direction:', direction);

    try
    {
        // Use enhanced detection with dual-stage trigger support
        const result = await detectInputWithDualStageSupport(thisSessionId, 10);

        // Check if this session is still active
        if (currentHatDetectionSessionId !== thisSessionId)
        {
            console.log('[HAT-DETECTION] Session', thisSessionId, 'cancelled, ignoring result');
            return;
        }

        if (result)
        {
            console.log('[HAT-DETECTION] Session', thisSessionId, 'detected input for', direction, ':', result);
            console.log('Input string:', result.input_string);

            // The Rust backend now returns proper Star Citizen format
            // Examples: "js1_hat1_up", "js1_button3", "js2_axis2"

            // Get the current page's configuration
            const currentPage = getCurrentPage();
            const currentStickData = currentPage || (currentStick === 'left' ? templateData.leftStick : templateData.rightStick);
            const templateJsNum = (currentStickData && currentStickData.joystickNumber) || templateData.joystickNumber || 1;

            // For pages: use device_uuid to validate, for legacy sticks: use detectedJsNumber
            const pageDeviceUuid = currentPage?.device_uuid;
            const detectedDeviceUuid = result.device_uuid;

            // Validate that input is from the configured device (if configured)
            if (pageDeviceUuid && detectedDeviceUuid && pageDeviceUuid !== detectedDeviceUuid)
            {
                console.warn(`Input from device ${detectedDeviceUuid} but expected ${pageDeviceUuid}`);
                const pageName = currentPage?.name || 'this page';
                document.getElementById('hat-detection-status').textContent =
                    `⚠️ That input is from a device not assigned to ${pageName}. Please use the device you configured for this page.`;
                document.getElementById('hat-detection-status').style.color = '#f0ad4e';
                return;
            }

            let adjustedInputString = result.input_string;

            // Replace jsX_ with the current stick's template joystick number (js1 or js2)
            // OR use the page's explicit prefix if set (e.g. "js1", "gp1")
            if (currentPage && currentPage.joystick_prefix)
            {
                adjustedInputString = adjustedInputString.replace(/^(js|gp)\d+_/, `${currentPage.joystick_prefix}_`);
                console.log(`Adjusted input string for ${direction} using page prefix ${currentPage.joystick_prefix}:`, adjustedInputString);
            }
            else
            {
                adjustedInputString = adjustedInputString.replace(/^(js|gp)\d+_/, `js${templateJsNum}_`);
                console.log('Adjusted input string for', direction, '(remapped to template js number):', adjustedInputString);
            }

            // Convert to Star Citizen axis format using HID axis name from backend if it's an axis
            // Skip hat switches - they should be converted by backend already
            if (result.hid_axis_name && adjustedInputString.includes('_axis'))
            {
                const hidName = result.hid_axis_name.toLowerCase().replace(/\s+/g, '');

                // Skip hat switches - they need special handling by the backend
                if (hidName === 'hatswitch')
                {
                    console.log(`Hat ${direction}: Skipping HID axis conversion for hat switch - should already be in hat format`);
                }
                else
                {
                    // Convert HID axis name to lowercase SC format (Rz -> rotz, X -> x, etc.)
                    const scAxisName = hidName === 'rx' ? 'rotx' :
                        hidName === 'ry' ? 'roty' :
                            hidName === 'rz' ? 'rotz' : hidName;

                    // Replace axis number format with axis name format
                    adjustedInputString = adjustedInputString.replace(/axis\d+(?:_(positive|negative))?/, scAxisName);
                    console.log(`Hat ${direction}: Converted to SC format using HID axis name "${result.hid_axis_name}":`, adjustedInputString);
                }
            }

            // Store the adjusted Star Citizen input string in tempButton
            if (tempButton)
            {
                if (!tempButton.inputs)
                {
                    tempButton.inputs = {};
                }

                // Store the complete SC format string (e.g., "js1_hat1_up" or "js1_button15")
                tempButton.inputs[direction] = adjustedInputString;

                // Update display to show the detected input
                const match = adjustedInputString.match(/button(\d+)/);
                if (match)
                {
                    const buttonId = parseInt(match[1]);
                    const display = document.querySelector(`[data-direction="${direction}"].hat-id-display`);
                    if (display)
                    {
                        display.textContent = `${buttonId} (${adjustedInputString})`;
                    }
                }
                else
                {
                    // For hat inputs, just show the full string
                    const display = document.querySelector(`[data-direction="${direction}"].hat-id-display`);
                    if (display)
                    {
                        display.textContent = adjustedInputString;
                    }
                }
            }

            // Use shared utility for display name
            const displayText = parseInputShortName(result.input_string);

            // Update button to show it's detected
            btn.textContent = `✓ (${displayText})`;

            document.getElementById('hat-detection-status').textContent = `${direction}: ${result.display_name}`;
            document.getElementById('hat-detection-status').style.color = '#5cb85c';

            // Clear any existing timeout
            if (hatDetectionTimeout !== null)
            {
                clearTimeout(hatDetectionTimeout);
            }

            hatDetectionTimeout = setTimeout(() =>
            {
                document.getElementById('hat-detection-status').style.display = 'none';
                detectingInput = false; // Clear the flag after the timeout
                hatDetectionTimeout = null;
            }, 2000);
        }
        else
        {
            btn.textContent = originalText;
            document.getElementById('hat-detection-status').textContent = 'Timeout - try again';
            document.getElementById('hat-detection-status').style.color = '#d9534f';
        }
    }
    catch (error)
    {
        console.error('Error detecting input:', error);
        btn.textContent = originalText;
        document.getElementById('hat-detection-status').textContent = `Error: ${error}`;
        document.getElementById('hat-detection-status').style.color = '#d9534f';
    }
    finally
    {
        // Only clear session if this was the active session
        if (currentHatDetectionSessionId === thisSessionId)
        {
            console.log('[HAT-DETECTION] Cleaning up session:', thisSessionId);
            currentHatDetectionSessionId = null;
        }
        detectingInput = false;
        btn.disabled = false;
    }
}

// Joystick Input Detection
async function startInputDetection()
{
    if (detectingInput)
    {
        stopInputDetection();
        return;
    }

    detectingInput = true;
    document.getElementById('button-modal-detect').textContent = 'Detecting...';
    document.getElementById('button-modal-detect').classList.add('btn-primary');
    document.getElementById('button-modal-detect').disabled = true;
    document.getElementById('input-detection-status').textContent = 'Press any button or move any axis on your joystick...';
    document.getElementById('input-detection-status').style.display = 'block';

    // Generate unique session ID for this detection
    const thisSessionId = Date.now() + Math.random();
    currentDetectionSessionId = thisSessionId;
    console.log('[INPUT-DETECTION] Starting detection session:', thisSessionId);

    try
    {
        // Use enhanced detection with dual-stage trigger support
        const result = await detectInputWithDualStageSupport(thisSessionId, 10);

        // Check if this session is still active
        if (currentDetectionSessionId !== thisSessionId)
        {
            console.log('[INPUT-DETECTION] Session', thisSessionId, 'cancelled, ignoring result');
            return;
        }

        if (result)
        {
            console.log('[INPUT-DETECTION] Session', thisSessionId, 'detected input:', result);
            console.log('Input string:', result.input_string);

            // The Rust backend now returns proper Star Citizen format
            // Examples: "js1_hat1_up", "js1_button3", "js2_axis2"

            // Get the current page's configuration
            const currentPage = getCurrentPage();
            const currentStickData = currentPage || (currentStick === 'left' ? templateData.leftStick : templateData.rightStick);
            const templateJsNum = (currentStickData && currentStickData.joystickNumber) || templateData.joystickNumber || 1;

            // For pages: use device_uuid to validate, for legacy sticks: use detectedJsNumber
            const pageDeviceUuid = currentPage?.device_uuid;
            const detectedDeviceUuid = result.device_uuid;

            // Validate that input is from the configured device (if configured)
            if (pageDeviceUuid && detectedDeviceUuid && pageDeviceUuid !== detectedDeviceUuid)
            {
                console.warn(`Input from device ${detectedDeviceUuid} but expected ${pageDeviceUuid}`);
                const pageName = currentPage?.name || 'this page';
                document.getElementById('input-detection-status').textContent =
                    `⚠️ That input is from a device not assigned to ${pageName}. Please use the device you configured for this page.`;
                document.getElementById('input-detection-status').style.color = '#f0ad4e';
                return;
            }

            let adjustedInputString = result.input_string;

            // Replace jsX_ with the current stick's template joystick number (js1 or js2)
            // OR use the page's explicit prefix if set (e.g. "js1", "gp1")
            if (currentPage && currentPage.joystick_prefix)
            {
                adjustedInputString = adjustedInputString.replace(/^(js|gp)\d+_/, `${currentPage.joystick_prefix}_`);
                console.log(`Adjusted input string using page prefix ${currentPage.joystick_prefix}:`, adjustedInputString);
            }
            else
            {
                adjustedInputString = adjustedInputString.replace(/^(js|gp)\d+_/, `js${templateJsNum}_`);
                console.log('Adjusted input string (remapped to template js number):', adjustedInputString);
            }

            // Convert to Star Citizen axis format using HID axis name from backend (e.g., js1_axis2 -> js1_rz)
            // This ensures we use the actual axis name from the HID descriptor, not hardcoded mappings
            // Skip hat switches - they should be converted by backend already
            if (result.hid_axis_name && adjustedInputString.includes('_axis'))
            {
                const hidName = result.hid_axis_name.toLowerCase().replace(/\s+/g, '');

                // Skip hat switches - they need special handling by the backend
                if (hidName === 'hatswitch')
                {
                    console.log('Skipping HID axis conversion for hat switch - should already be in hat format');
                }
                else
                {
                    // Convert HID axis name to lowercase SC format (Rz -> rotz, X -> x, etc.)
                    const scAxisName = hidName === 'rx' ? 'rotx' :
                        hidName === 'ry' ? 'roty' :
                            hidName === 'rz' ? 'rotz' : hidName;

                    // Replace axis number format with axis name format
                    adjustedInputString = adjustedInputString.replace(/axis\d+(?:_(positive|negative))?/, scAxisName);
                    console.log(`Converted to Star Citizen format using HID axis name "${result.hid_axis_name}":`, adjustedInputString);
                }
            }
            else if (adjustedInputString.includes('_axis'))
            {
                // Fallback: use hardcoded mapping if no HID axis name available (for XInput gamepads)
                const scFormatString = toStarCitizenFormat(adjustedInputString);
                if (scFormatString)
                {
                    adjustedInputString = scFormatString;
                    console.log('Converted to Star Citizen format using default mapping:', adjustedInputString);
                }
            }

            // Use shared utility for friendly name (use adjusted string)
            const inputName = parseInputDisplayName(adjustedInputString);

            // Update the input field with a friendly name only if empty
            const buttonNameInput = document.getElementById('button-name-input');
            if (!buttonNameInput.value)
            {
                buttonNameInput.value = inputName;
            }

            // Store the adjusted Star Citizen format string in tempButton
            if (tempButton)
            {
                tempButton.buttonType = 'simple';
                // Only set name if it's currently empty
                if (!tempButton.name)
                {
                    tempButton.name = inputName;
                }

                if (!tempButton.inputs)
                {
                    tempButton.inputs = {};
                }

                tempButton.inputs.main = adjustedInputString;

                const buttonMatch = adjustedInputString.match(/button(\d+)/);
                const axisNumericMatch = adjustedInputString.match(/axis(\d+)(?:_(positive|negative))?/);
                const axisSCMatch = adjustedInputString.match(/^(js|gp)\d+_(x|y|z|rotx|roty|rotz|slider)$/);
                const hatMatch = adjustedInputString.match(/hat(\d+)_(up|down|left|right)/);

                if (hatMatch)
                {
                    // Hat switch input (e.g., js2_hat1_up)
                    const hatNum = parseInt(hatMatch[1]);
                    const hatDirection = hatMatch[2];
                    delete tempButton.buttonId;
                    tempButton.inputType = 'hat';
                    tempButton.inputId = hatNum;
                    tempButton.hatDirection = hatDirection;
                    delete tempButton.axisDirection;
                }
                else if (buttonMatch)
                {
                    const buttonId = parseInt(buttonMatch[1]);
                    tempButton.buttonId = buttonId;
                    tempButton.inputType = 'button';
                    tempButton.inputId = buttonId;
                    delete tempButton.axisDirection;
                }
                else if (axisNumericMatch)
                {
                    const axisId = parseInt(axisNumericMatch[1]);
                    delete tempButton.buttonId;
                    tempButton.inputType = 'axis';
                    tempButton.inputId = axisId;
                    tempButton.axisDirection = axisNumericMatch[2] || null;
                }
                else if (axisSCMatch)
                {
                    // Star Citizen axis format (e.g., js1_x, js1_y)
                    delete tempButton.buttonId;
                    tempButton.inputType = 'axis';
                    tempButton.inputId = axisSCMatch[2]; // Store the axis name (x, y, z, etc.)
                    delete tempButton.axisDirection;
                }
                else
                {
                    delete tempButton.buttonId;
                    delete tempButton.axisDirection;
                    tempButton.inputType = 'input';
                    tempButton.inputId = undefined;
                }

                updateSimpleInputPreview(tempButton);
            }

            // Show confirmation
            document.getElementById('input-detection-status').textContent = `Detected: ${result.display_name}`;
            document.getElementById('input-detection-status').style.color = '#5cb85c';

            // Clear any existing timeout
            if (inputDetectionTimeout !== null)
            {
                clearTimeout(inputDetectionTimeout);
            }

            inputDetectionTimeout = setTimeout(() =>
            {
                document.getElementById('input-detection-status').style.display = 'none';
                document.getElementById('input-detection-status').style.color = '';
                detectingInput = false; // Clear the flag after the timeout
                inputDetectionTimeout = null;
            }, 2000);
        }
        else
        {
            // Timeout
            document.getElementById('input-detection-status').textContent = 'No input detected - timed out';
            document.getElementById('input-detection-status').style.color = '#d9534f';

            // Clear any existing timeout
            if (inputDetectionTimeout !== null)
            {
                clearTimeout(inputDetectionTimeout);
            }

            inputDetectionTimeout = setTimeout(() =>
            {
                document.getElementById('input-detection-status').style.display = 'none';
                document.getElementById('input-detection-status').style.color = '';
                inputDetectionTimeout = null;
            }, 3000);
        }
    }
    catch (error)
    {
        console.error('Error detecting input:', error);
        document.getElementById('input-detection-status').textContent = `Error: ${error}`;
        document.getElementById('input-detection-status').style.color = '#d9534f';
    }
    finally
    {
        // Only clear session if this was the active session
        if (currentDetectionSessionId === thisSessionId)
        {
            console.log('[INPUT-DETECTION] Cleaning up session:', thisSessionId);
            currentDetectionSessionId = null;
        }
        detectingInput = false;
        document.getElementById('button-modal-detect').textContent = '🎮 Detect Input';
        document.getElementById('button-modal-detect').classList.remove('btn-primary');
        document.getElementById('button-modal-detect').disabled = false;
    }
}

function stopInputDetection()
{
    // Clear any pending timeouts
    if (inputDetectionTimeout !== null)
    {
        clearTimeout(inputDetectionTimeout);
        inputDetectionTimeout = null;
    }

    if (hatDetectionTimeout !== null)
    {
        clearTimeout(hatDetectionTimeout);
        hatDetectionTimeout = null;
    }

    // Clear session IDs to invalidate any pending operations
    console.log('[INPUT-DETECTION] Stopping detection, clearing sessions');
    currentDetectionSessionId = null;
    currentHatDetectionSessionId = null;

    detectingInput = false;
    document.getElementById('button-modal-detect').textContent = '🎮 Detect Input';
    document.getElementById('button-modal-detect').classList.remove('btn-primary');
    document.getElementById('button-modal-detect').disabled = false;
    document.getElementById('input-detection-status').style.display = 'none';
}

// Clear simple button input
function clearSimpleButtonInput()
{
    if (!tempButton) return;

    tempButton.inputs = {};
    tempButton.buttonId = undefined;
    delete tempButton.inputType;
    delete tempButton.inputId;
    delete tempButton.axisDirection;
    updateSimpleInputPreview(tempButton);
    document.getElementById('input-detection-status').style.display = 'none';

    markAsChanged();
}

// Clear hat direction input
function clearHatDirection(direction)
{
    if (!tempButton) return;

    if (tempButton.inputs)
    {
        delete tempButton.inputs[direction];
    }

    const display = document.querySelector(`[data-direction="${direction}"].hat-id-display`);
    if (display)
    {
        display.textContent = '—';
    }

    const btn = document.querySelector(`[data-direction="${direction}"].hat-detect-btn`);
    if (btn)
    {
        const direction_label = direction.charAt(0).toUpperCase() + direction.slice(1);
        const emoji = { up: '⬆️', down: '⬇️', left: '⬅️', right: '➡️', push: '⬇️' }[direction];
        btn.textContent = `${emoji} Detect ${direction_label}`;
        btn.classList.remove('btn-primary');
        btn.classList.add('btn-secondary');
    }

    markAsChanged();
}

// Template save/load
// Helper function to prepare save data
function prepareSaveData()
{
    // Helper to extract buttons array from stick (handles nested or flat structure)
    const getStickButtons = (stick) =>
    {
        if (Array.isArray(stick)) return stick;
        if (stick && stick.buttons && Array.isArray(stick.buttons)) return stick.buttons;
        return [];
    };

    // Prepare data for saving with nested structure
    return {
        name: templateData.name,
        joystickModel: templateData.joystickModel,
        version: templateData.version || '1.0',
        imageWidth: loadedImage ? loadedImage.width : 0,
        imageHeight: loadedImage ? loadedImage.height : 0,
        leftStick: {
            joystickNumber: templateData.leftStick.joystickNumber || 1,
            buttons: getStickButtons(templateData.leftStick).map(b => ({
                id: b.id,
                name: b.name,
                buttonPos: b.buttonPos,
                labelPos: b.labelPos,
                buttonType: b.buttonType || 'simple',
                inputs: b.inputs || {},
                // Legacy support
                inputType: b.inputType,
                inputId: b.inputId
            }))
        },
        rightStick: {
            joystickNumber: templateData.rightStick.joystickNumber || 2,
            buttons: getStickButtons(templateData.rightStick).map(b => ({
                id: b.id,
                name: b.name,
                buttonPos: b.buttonPos,
                labelPos: b.labelPos,
                buttonType: b.buttonType || 'simple',
                inputs: b.inputs || {},
                // Legacy support
                inputType: b.inputType,
                inputId: b.inputId
            }))
        },
        pages: Array.isArray(templateData.pages) ? templateData.pages.map(page => ({
            id: page.id,
            name: page.name || 'Untitled Page',
            device_uuid: page.device_uuid || '',
            device_name: page.device_name || '',
            joystickNumber: page.joystickNumber || 1,
            joystick_prefix: page.joystick_prefix || '',
            axis_profile: page.axis_profile || 'default',
            axis_mapping: page.axis_mapping || {},
            image_path: page.image_path || '',
            image_data_url: page.image_data_url || null,
            mirror_from_page_id: page.mirror_from_page_id || '',
            buttons: (page.buttons || []).map(b => ({
                id: b.id,
                name: b.name,
                buttonPos: b.buttonPos,
                labelPos: b.labelPos,
                buttonType: b.buttonType || 'simple',
                inputs: b.inputs || {},
                inputType: b.inputType,
                inputId: b.inputId
            }))
        })) : []
    };
}

// Helper function to save template to a given file path
async function performSave(filePath, showNotification = true)
{
    const showAlert = window.showAlert || alert;

    try
    {
        const saveData = prepareSaveData();

        await invoke('save_template', {
            filePath,
            templateJson: JSON.stringify(saveData, null, 2)
        });

        // Update current file path for future saves
        currentTemplateFilePath = filePath;

        // Persist to localStorage
        localStorage.setItem('currentTemplate', JSON.stringify(saveData));
        const fileName = filePath.split(/[\\\/]/).pop();
        localStorage.setItem('templateFileName', fileName);
        localStorage.setItem('templateFilePath', filePath);

        // Clear unsaved changes
        hasUnsavedChanges = false;
        updateUnsavedIndicator();

        // Update header template name
        if (window.updateTemplateIndicator)
        {
            window.updateTemplateIndicator(templateData.name, fileName);
        }

        if (showNotification)
        {
            await showAlert('Template saved successfully!', 'Template Saved');
        }

        return true;
    } catch (error)
    {
        console.error('Error saving template:', error);
        await showAlert(`Failed to save template: ${error}`, 'Error');
        return false;
    }
}

// Save to current file (auto-save), or show dialog if no current file
async function saveTemplate()
{
    const showAlert = window.showAlert || alert;

    if (!templateData.name)
    {
        await showAlert('Please enter a template name', 'Missing Template Name');
        document.getElementById('template-name').focus();
        return;
    }

    // For dual image mode, require both images
    if (templateData.imageType === 'dual')
    {
        if (!templateData.leftImageDataUrl || !templateData.rightImageDataUrl)
        {
            await showAlert('Please load images for both left and right sticks', 'Images Required');
            return;
        }
    }

    // Count buttons from nested structure
    const leftButtons = getCurrentButtons();
    const rightButtons = currentStick === 'left' ?
        (templateData.rightStick.buttons || templateData.rightStick || []) :
        (templateData.leftStick.buttons || templateData.leftStick || []);
    const totalButtons = leftButtons.length + (Array.isArray(rightButtons) ? rightButtons.length : 0);

    if (totalButtons === 0)
    {
        await showAlert('Please add at least one button to either stick', 'No Buttons Defined');
        return;
    }

    // If we have a current file path, save directly without dialog
    if (currentTemplateFilePath)
    {
        await performSave(currentTemplateFilePath, true);
        return;
    }

    // Otherwise, show file picker (same as Save As)
    try
    {
        let resourceDir;
        try
        {
            resourceDir = await invoke('get_resource_dir');
        }
        catch (e)
        {
            console.warn('Could not get resource directory:', e);
            resourceDir = undefined;
        }

        const filePath = await save({
            filters: [{
                name: 'Joystick Template',
                extensions: ['json']
            }],
            defaultPath: resourceDir
        });

        if (!filePath) return; // User cancelled

        await performSave(filePath, true);
    } catch (error)
    {
        console.error('Error saving template:', error);
        await showAlert(`Failed to save template: ${error}`, 'Error');
    }
}

// Save As - always shows file picker
async function saveTemplateAs()
{
    const showAlert = window.showAlert || alert;

    if (!templateData.name)
    {
        await showAlert('Please enter a template name', 'Missing Template Name');
        document.getElementById('template-name').focus();
        return;
    }

    if (!loadedImage)
    {
        await showAlert('Please load a joystick image', 'No Image Loaded');
        return;
    }

    // For dual image mode, require both images
    if (templateData.imageType === 'dual')
    {
        if (!templateData.leftImageDataUrl || !templateData.rightImageDataUrl)
        {
            await showAlert('Please load images for both left and right sticks', 'Images Required');
            return;
        }
    }

    // Count buttons from nested structure
    const leftButtons = getCurrentButtons();
    const rightButtons = currentStick === 'left' ?
        (templateData.rightStick.buttons || templateData.rightStick || []) :
        (templateData.leftStick.buttons || templateData.leftStick || []);
    const totalButtons = leftButtons.length + (Array.isArray(rightButtons) ? rightButtons.length : 0);

    if (totalButtons === 0)
    {
        await showAlert('Please add at least one button to either stick', 'No Buttons Defined');
        return;
    }

    try
    {
        let resourceDir;
        try
        {
            resourceDir = await invoke('get_resource_dir');
        }
        catch (e)
        {
            console.warn('Could not get resource directory:', e);
            resourceDir = undefined;
        }

        // Always show file picker for Save As
        const filePath = await save({
            filters: [{
                name: 'Joystick Template',
                extensions: ['json']
            }],
            defaultPath: resourceDir
        });

        if (!filePath) return; // User cancelled

        await performSave(filePath, true);
    } catch (error)
    {
        console.error('Error saving template:', error);
        await showAlert(`Failed to save template: ${error}`, 'Error');
    }
}

async function loadTemplate()
{
    try
    {
        let defaultPath;
        try
        {
            defaultPath = await invoke('get_resource_dir');
        }
        catch (e)
        {
            console.warn('Could not get resource directory:', e);
        }

        const filePath = await open({
            filters: [{
                name: 'Joystick Template',
                extensions: ['json']
            }],
            multiple: false,
            defaultPath: defaultPath
        });

        if (!filePath) return; // User cancelled

        const templateJson = await invoke('load_template', { filePath });
        const data = JSON.parse(templateJson);

        // Load the data - handle both old and new formats
        templateData.name = data.name || '';
        templateData.joystickModel = data.joystickModel || '';
        templateData.joystickNumber = data.joystickNumber || 1;

        // Handle buttons: support multiple formats
        // Format 1: New nested format { leftStick: { joystickNumber: 1, buttons: [...] }, rightStick: { joystickNumber: 2, buttons: [...] } }
        // Format 2: Flat array format { leftStick: [...], rightStick: [...] }
        // Format 3: Old single stick format { buttons: [...] }

        if (data.leftStick || data.rightStick)
        {
            // New dual stick format (nested or flat)
            templateData.leftStick = data.leftStick || { joystickNumber: 1, buttons: [] };
            templateData.rightStick = data.rightStick || { joystickNumber: 2, buttons: [] };

            // Ensure nested structure has buttons array
            if (templateData.leftStick && typeof templateData.leftStick === 'object' && !Array.isArray(templateData.leftStick) && !templateData.leftStick.buttons)
            {
                templateData.leftStick.buttons = [];
            }
            if (templateData.rightStick && typeof templateData.rightStick === 'object' && !Array.isArray(templateData.rightStick) && !templateData.rightStick.buttons)
            {
                templateData.rightStick.buttons = [];
            }
        }
        else if (data.buttons)
        {
            // Old single stick format - put all buttons in right stick by default
            templateData.leftStick = { joystickNumber: 1, buttons: [] };
            templateData.rightStick = { joystickNumber: 2, buttons: data.buttons || [] };
        }
        else
        {
            // No buttons at all
            templateData.leftStick = { joystickNumber: 1, buttons: [] };
            templateData.rightStick = { joystickNumber: 2, buttons: [] };
        }
        templateData.version = data.version || '1.0';
        templateData.pages = Array.isArray(data.pages) ? data.pages : [];
        ensureTemplatePages();
        refreshTemplatePagesUI(templateData);

        // Persist to localStorage
        localStorage.setItem('currentTemplate', JSON.stringify(data));
        const fileName = filePath.split(/[\\\/]/).pop();
        localStorage.setItem('templateFileName', fileName);
        localStorage.setItem('templateFilePath', filePath);

        // Set current file path for auto-save
        currentTemplateFilePath = filePath;

        // Reset unsaved changes
        hasUnsavedChanges = false;
        updateUnsavedIndicator();

        // Update header template name
        console.log('loadTemplate - data.name:', data.name);
        console.log('window.updateTemplateIndicator exists:', typeof window.updateTemplateIndicator);
        if (window.updateTemplateIndicator)
        {
            console.log('Calling updateTemplateIndicator with:', data.name || 'Untitled Template', fileName);
            window.updateTemplateIndicator(data.name || 'Untitled Template', fileName);
        }
        else
        {
            console.log('window.updateTemplateIndicator is not available');
        }

        // Update UI
        document.getElementById('template-name').value = templateData.name;
        document.getElementById('joystick-model').value = templateData.joystickModel;

        // Load the first page's image if we have pages
        if (currentPageId && templateData.pages.length > 0)
        {
            const firstPage = templateData.pages.find(p => p.id === currentPageId);
            if (firstPage)
            {
                loadPageImage(firstPage);
                updateButtonList();
            }
        }
        // Legacy image handling for backward compatibility (only if no pages)
        else if (templateData.imageType === 'dual' && templateData.leftImageDataUrl)
        {
            const img = new Image();
            img.onload = () =>
            {
                resizeImage(img, 1024, (resizedImg) =>
                {
                    loadedImage = resizedImg;
                    resizeCanvas();
                    requestAnimationFrame(() =>
                    {
                        fitToScreen();
                        updateButtonList();
                    });
                });
            };
            img.src = templateData.leftImageDataUrl;
        }
        else if (templateData.imageDataUrl)
        {
            const img = new Image();
            img.onload = () =>
            {
                resizeImage(img, 1024, (resizedImg) =>
                {
                    loadedImage = resizedImg;
                    resizeCanvas();
                    requestAnimationFrame(() =>
                    {
                        fitToScreen();
                        updateButtonList();
                    });
                });
            };
            img.src = templateData.imageDataUrl;
        }

    } catch (error)
    {
        console.error('Error loading template:', error);
        const showAlert = window.showAlert || alert;
        await showAlert(`Failed to load template: ${error}`, 'Error');
    }
}

// Helper functions for hat detection buttons
function resetHatDetectionButtons()
{
    document.querySelectorAll('.hat-detect-btn').forEach(btn =>
    {
        const direction = btn.dataset.direction;
        const emoji = { up: '⬆️', down: '⬇️', left: '⬅️', right: '➡️', push: '⬇️' }[direction];
        btn.textContent = `${emoji} Detect ${direction.charAt(0).toUpperCase() + direction.slice(1)}`;
        btn.classList.remove('btn-primary');
        btn.classList.add('btn-secondary');
    });
    document.getElementById('hat-detection-status').style.display = 'none';
}

function updateHatDetectionButtons(inputs)
{
    Object.keys(inputs).forEach(direction =>
    {
        const input = inputs[direction];
        const btn = document.querySelector(`[data-direction="${direction}"].hat-detect-btn`);
        if (btn && input)
        {
            const emoji = { up: '⬆️', down: '⬇️', left: '⬅️', right: '➡️', push: '⬇️' }[direction];

            // Handle both string format (js1_button3) and object format ({id: 3})
            let displayText = '';
            if (typeof input === 'string')
            {
                displayText = parseInputShortName(input);
            }
            else if (typeof input === 'object' && input.id !== undefined)
            {
                displayText = `Btn ${input.id}`;
            }
            else
            {
                return; // Skip if we can't parse it
            }

            btn.textContent = `${emoji} ✓ (${displayText})`;
        }
    });
}

function updateFileIndicator()
{
    const indicator = document.getElementById('loaded-file-indicator');
    const fileNameEl = document.getElementById('loaded-file-name');
    const savedPath = localStorage.getItem('keybindingsFilePath');

    if (indicator && fileNameEl && savedPath)
    {
        const fileName = savedPath.split(/[\\\\/]/).pop();
        fileNameEl.textContent = fileName;
        indicator.style.display = 'flex';
    }
}

function updateUnsavedIndicator()
{
    const indicator = document.getElementById('loaded-file-indicator');
    const fileNameEl = document.getElementById('loaded-file-name');

    if (indicator && fileNameEl)
    {
        if (typeof hasUnsavedChanges !== 'undefined' && hasUnsavedChanges)
        {
            indicator.style.borderColor = 'var(--accent-primary)';
            indicator.style.backgroundColor = 'rgba(217, 83, 79, 0.1)';
            if (!fileNameEl.textContent.includes('*'))
            {
                fileNameEl.textContent += ' *';
            }
        }
        else
        {
            indicator.style.borderColor = 'var(--button-border)';
            indicator.style.backgroundColor = 'var(--bg-medium)';
            fileNameEl.textContent = fileNameEl.textContent.replace(' *', '');
        }
    }
}

function loadPersistedTemplate()
{
    try
    {
        const savedTemplate = localStorage.getItem('currentTemplate');
        if (savedTemplate)
        {
            const data = JSON.parse(savedTemplate);

            // Restore file path for auto-save functionality
            const savedFilePath = localStorage.getItem('templateFilePath');
            if (savedFilePath)
            {
                currentTemplateFilePath = savedFilePath;
            }

            // Restore camera positions if available
            const savedLeftCamera = localStorage.getItem('leftStickCamera');
            const savedRightCamera = localStorage.getItem('rightStickCamera');

            if (savedLeftCamera)
            {
                leftStickCamera = JSON.parse(savedLeftCamera);
            }
            if (savedRightCamera)
            {
                rightStickCamera = JSON.parse(savedRightCamera);
            }

            // Load the data
            templateData.name = data.name || '';
            templateData.joystickModel = data.joystickModel || '';
            templateData.joystickNumber = data.joystickNumber || 1;
            templateData.imagePath = data.imagePath || '';
            templateData.imageDataUrl = data.imageDataUrl || null;

            // Handle imageType
            templateData.imageType = data.imageType || 'single';

            // Handle dual image data
            templateData.leftImagePath = data.leftImagePath || '';
            templateData.leftImageDataUrl = data.leftImageDataUrl || null;
            templateData.rightImagePath = data.rightImagePath || '';
            templateData.rightImageDataUrl = data.rightImageDataUrl || null;

            // Handle imageFlipped: convert old boolean format to new format
            if (typeof data.imageFlipped === 'boolean')
            {
                templateData.imageFlipped = data.imageFlipped ? 'left' : 'right';
            }
            else
            {
                templateData.imageFlipped = data.imageFlipped || 'right';
            }

            // Handle buttons: support multiple formats
            // Format 1: New nested format { leftStick: { joystickNumber: 1, buttons: [...] }, rightStick: { joystickNumber: 2, buttons: [...] } }
            // Format 2: Flat array format { leftStick: [...], rightStick: [...] }
            // Format 3: Old single stick format { buttons: [...] }

            if (data.leftStick || data.rightStick)
            {
                // New dual stick format (nested or flat)
                templateData.leftStick = data.leftStick || { joystickNumber: 1, buttons: [] };
                templateData.rightStick = data.rightStick || { joystickNumber: 2, buttons: [] };

                // Ensure nested structure has buttons array
                if (templateData.leftStick && typeof templateData.leftStick === 'object' && !Array.isArray(templateData.leftStick) && !templateData.leftStick.buttons)
                {
                    templateData.leftStick.buttons = [];
                }
                if (templateData.rightStick && typeof templateData.rightStick === 'object' && !Array.isArray(templateData.rightStick) && !templateData.rightStick.buttons)
                {
                    templateData.rightStick.buttons = [];
                }
            }
            else if (data.buttons)
            {
                // Old single stick format - put all buttons in right stick by default
                templateData.leftStick = { joystickNumber: 1, buttons: [] };
                templateData.rightStick = { joystickNumber: 2, buttons: data.buttons || [] };
            }
            else
            {
                // No buttons at all
                templateData.leftStick = { joystickNumber: 1, buttons: [] };
                templateData.rightStick = { joystickNumber: 2, buttons: [] };
            }

            templateData.version = data.version || '1.0';
            templateData.pages = Array.isArray(data.pages) ? data.pages : [];
            ensureTemplatePages();
            refreshTemplatePagesUI(templateData);

            // Update UI
            document.getElementById('template-name').value = templateData.name;
            document.getElementById('joystick-model').value = templateData.joystickModel;

            // Load the first page's image if we have pages
            if (currentPageId && templateData.pages.length > 0)
            {
                const firstPage = templateData.pages.find(p => p.id === currentPageId);
                if (firstPage)
                {
                    loadPageImage(firstPage);
                    updateButtonList();
                }
            }
            // Legacy image handling for backward compatibility (only if no pages)
            else if (templateData.imageType === 'dual' && templateData.leftImageDataUrl)
            {
                const img = new Image();
                img.onload = () =>
                {
                    resizeImage(img, 1024, (resizedImg) =>
                    {
                        loadedImage = resizedImg;
                        resizeCanvas();
                        requestAnimationFrame(() =>
                        {
                            fitToScreen();
                            updateButtonList();
                        });
                    });
                };
                img.src = templateData.leftImageDataUrl;
            }
            else if (templateData.imageDataUrl)
            {
                const img = new Image();
                img.onload = () =>
                {
                    resizeImage(img, 1024, (resizedImg) =>
                    {
                        loadedImage = resizedImg;
                        resizeCanvas();
                        requestAnimationFrame(() =>
                        {
                            fitToScreen();
                            updateButtonList();
                        });
                    });
                };
                img.src = templateData.imageDataUrl;
            }
        }
    } catch (error)
    {
        console.error('Error loading persisted template:', error);
    }
}

function markAsChanged()
{
    hasUnsavedChanges = true;
    updateUnsavedIndicator();

    // Also persist to localStorage for recovery
    try
    {
        localStorage.setItem('currentTemplate', JSON.stringify(templateData));
        // Also persist camera positions for each stick
        localStorage.setItem('leftStickCamera', JSON.stringify(leftStickCamera));
        localStorage.setItem('rightStickCamera', JSON.stringify(rightStickCamera));
    } catch (error)
    {
        console.error('Error persisting template changes:', error);
    }
}

window.markTemplateAsChanged = markAsChanged;

// ============================================================================
// TEMPLATE JOYSTICK MAPPING
// ============================================================================

