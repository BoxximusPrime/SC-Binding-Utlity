// Character Manager Module
// Manages SC character appearance backups and deployments

const { invoke } = window.__TAURI__.core;
const { open } = window.__TAURI__.dialog;

class CharacterManager
{
    constructor()
    {
        this.libraryPath = null;
        this.installations = [];
        this.activeInstallation = null;
        this.masterCharacters = [];
        this.installationCharacters = {};

        this.init();
    }

    async init()
    {
        // Load saved library path
        this.libraryPath = localStorage.getItem('characterLibraryPath');

        // Load SC installations first
        await this.loadInstallations();

        // Then load master characters (so sync status can be checked)
        if (this.libraryPath)
        {
            this.updateLibraryPathDisplay();
            await this.loadMasterCharacters();
        }

        // Setup event listeners
        this.setupEventListeners();
    }

    setupEventListeners()
    {
        // Set Library Path button
        document.getElementById('set-library-path-btn')?.addEventListener('click', async () =>
        {
            await this.selectLibraryPath();
        });

        // Refresh All button
        document.getElementById('refresh-characters-btn')?.addEventListener('click', async () =>
        {
            await this.refreshAll();
        });
    }

    async refreshAll()
    {
        this.showNotification('Refreshing character data...', 'info');

        try
        {
            // Reload master characters
            await this.loadMasterCharacters();

            // Reload all installations
            await this.loadInstallations();

            this.showSuccess('Character data refreshed successfully');
        } catch (error)
        {
            console.error('Error refreshing character data:', error);
            this.showError('Failed to refresh character data: ' + error);
        }
    }

    async selectLibraryPath()
    {
        try
        {
            const selectedPath = await open({
                directory: true,
                multiple: false,
                title: 'Select Character Library Directory'
            });

            if (selectedPath)
            {
                this.libraryPath = selectedPath;
                localStorage.setItem('characterLibraryPath', selectedPath);
                this.updateLibraryPathDisplay();
                await this.loadMasterCharacters();
            }
        } catch (error)
        {
            console.error('Error selecting library path:', error);
            this.showError('Failed to select library path: ' + error);
        }
    }

    updateLibraryPathDisplay()
    {
        const pathValueEl = document.getElementById('library-path-value');
        if (pathValueEl)
        {
            if (this.libraryPath)
            {
                pathValueEl.textContent = this.libraryPath;
                pathValueEl.classList.remove('empty');
            } else
            {
                pathValueEl.textContent = 'Not configured';
                pathValueEl.classList.add('empty');
            }
        }
    }

    async loadMasterCharacters()
    {
        if (!this.libraryPath)
        {
            this.renderEmptyMasterLibrary();
            return;
        }

        try
        {
            const characters = await invoke('scan_character_files', {
                directoryPath: this.libraryPath
            });

            this.masterCharacters = characters;
            this.renderMasterCharacters();
        } catch (error)
        {
            console.error('Error loading master characters:', error);
            this.renderEmptyMasterLibrary('Error loading characters: ' + error);
        }
    }

    async loadInstallations()
    {
        const scDirectory = localStorage.getItem('scInstallDirectory');
        if (!scDirectory)
        {
            this.renderEmptyInstallations();
            return;
        }

        try
        {
            const installations = await invoke('scan_sc_installations', {
                basePath: scDirectory
            });

            this.installations = installations;

            if (installations.length > 0)
            {
                // Load characters for each installation
                for (const install of installations)
                {
                    await this.loadInstallationCharacters(install);
                }

                this.renderInstallationTabs();

                // Activate first installation by default
                if (this.installations.length > 0)
                {
                    this.switchInstallation(this.installations[0].name);
                }
            } else
            {
                this.renderEmptyInstallations();
            }
        } catch (error)
        {
            console.error('Error loading installations:', error);
            this.renderEmptyInstallations('Error loading installations: ' + error);
        }
    }

    async loadInstallationCharacters(installation)
    {
        try
        {
            // Character path: INSTALL\user\client\0\customcharacters\
            const characterPath = `${installation.path}\\user\\client\\0\\customcharacters`;

            const characters = await invoke('scan_character_files', {
                directoryPath: characterPath
            });

            this.installationCharacters[installation.name] = characters;
        } catch (error)
        {
            console.error(`Error loading characters for ${installation.name}:`, error);
            this.installationCharacters[installation.name] = [];
        }
    }

    renderMasterCharacters()
    {
        const listEl = document.getElementById('master-characters-list');
        if (!listEl) return;

        if (this.masterCharacters.length === 0)
        {
            listEl.innerHTML = `
        <div class="empty-state">
          <div class="empty-state-icon">📭</div>
          <h3>No Characters Found</h3>
          <p>No .chf files found in the library directory</p>
        </div>
      `;
            return;
        }

        listEl.innerHTML = this.masterCharacters.map(char =>
        {
            const syncStatus = this.getMasterCharacterSyncStatus(char);
            const hasNewerVersion = this.hasNewerVersionInInstallations(char);

            return `
      <div class="character-card">
        <div class="character-icon">👤</div>
        <div class="character-info">
          <h4 class="character-name">${char.name}</h4>
          <div class="character-meta">
            <span>📅 ${this.formatDate(char.modified)}</span>
            <span>💾 ${this.formatFileSize(char.size)}</span>
          </div>
          ${syncStatus ? `<div class="character-sync-status">${syncStatus}</div>` : ''}
        </div>
        <div class="character-actions">
          ${hasNewerVersion ? `
            <button class="btn btn-primary btn-sm" onclick="characterManager.updateFromNewest('${char.name}')" title="Update library with newest version">
              🔄 Update
            </button>
          ` : ''}
          <button class="btn btn-secondary btn-sm" onclick="characterManager.exportCharacter('${char.name}')" title="Export to all installations">
            📤 Export All
          </button>
          <button class="btn btn-danger btn-sm" onclick="characterManager.deleteCharacter('${char.name}')" title="Delete from library">
            🗑️
          </button>
        </div>
      </div>
    `;
        }).join('');
    }

    hasNewerVersionInInstallations(masterChar)
    {
        for (const [installName, characters] of Object.entries(this.installationCharacters))
        {
            const installChar = characters.find(c => c.name === masterChar.name);
            if (installChar && installChar.modified > masterChar.modified)
            {
                return true;
            }
        }
        return false;
    }

    async updateFromNewest(characterName)
    {
        if (!this.libraryPath)
        {
            this.showError('Library path not configured');
            return;
        }

        try
        {
            // Find the newest version across all installations
            let newestVersion = null;
            let newestInstallation = null;
            let newestTimestamp = 0;

            for (const [installName, characters] of Object.entries(this.installationCharacters))
            {
                const installChar = characters.find(c => c.name === characterName);
                if (installChar && installChar.modified > newestTimestamp)
                {
                    newestVersion = installChar;
                    newestInstallation = this.installations.find(i => i.name === installName);
                    newestTimestamp = installChar.modified;
                }
            }

            if (!newestVersion || !newestInstallation)
            {
                this.showError('No newer version found in installations');
                return;
            }

            // Import the newest version to library
            await invoke('import_character_to_library', {
                characterName,
                installationPath: newestInstallation.path,
                libraryPath: this.libraryPath
            });

            this.showSuccess(`Updated ${characterName} in library from ${newestInstallation.name} (${this.formatDate(newestTimestamp)})`);

            // Reload master characters to reflect the update
            await this.loadMasterCharacters();

            // Optionally refresh the current installation view if it's active
            if (this.activeInstallation)
            {
                const activeInstall = this.installations.find(i => i.name === this.activeInstallation);
                if (activeInstall)
                {
                    await this.loadInstallationCharacters(activeInstall);
                    this.renderInstallationContent(this.activeInstallation);
                }
            }
        } catch (error)
        {
            console.error('Error updating from newest version:', error);
            this.showError('Failed to update from newest version: ' + error);
        }
    }

    getMasterCharacterSyncStatus(masterChar)
    {
        const statuses = [];
        let hasOutdated = false;
        let hasNewer = false;
        let hasMissing = false;

        // Check each installation for this character
        for (const [installName, characters] of Object.entries(this.installationCharacters))
        {
            const installChar = characters.find(c => c.name === masterChar.name);

            if (!installChar)
            {
                hasMissing = true;
            } else if (installChar.modified > masterChar.modified)
            {
                hasNewer = true;
            } else if (installChar.modified < masterChar.modified)
            {
                hasOutdated = true;
            }
        }

        // Build status message
        const parts = [];
        if (hasNewer)
        {
            parts.push('<span class="sync-status-newer">⚠️ Newer version in game</span>');
        }
        if (hasOutdated)
        {
            parts.push('<span class="sync-status-outdated">📤 Update available</span>');
        }
        if (hasMissing && this.installations.length > 0)
        {
            parts.push('<span class="sync-status-missing">📭 Not in all installations</span>');
        }

        return parts.length > 0 ? parts.join(' ') : null;
    }

    renderEmptyMasterLibrary(message = null)
    {
        const listEl = document.getElementById('master-characters-list');
        if (!listEl) return;

        listEl.innerHTML = `
      <div class="empty-state">
        <div class="empty-state-icon">📂</div>
        <h3>${message || 'No Character Library'}</h3>
        <p>${message ? '' : 'Set a library path to start managing your character appearances'}</p>
      </div>
    `;
    }

    renderInstallationTabs()
    {
        const tabsEl = document.getElementById('installation-tabs');
        if (!tabsEl) return;

        if (this.installations.length === 0)
        {
            this.renderEmptyInstallations();
            return;
        }

        const installationIcons = {
            'LIVE': '🌟',
            'PTU': '🧪',
            'EPTU': '🔬',
            'TECH-PREVIEW': '⚡'
        };

        tabsEl.innerHTML = this.installations.map(install => `
      <button class="installation-tab" data-install="${install.name}">
        <span class="installation-tab-icon">${installationIcons[install.name] || '🚀'}</span>
        <span>${install.name}</span>
      </button>
    `).join('');

        // Add click listeners
        tabsEl.querySelectorAll('.installation-tab').forEach(tab =>
        {
            tab.addEventListener('click', () =>
            {
                const installName = tab.dataset.install;
                this.switchInstallation(installName);
            });
        });
    }

    renderEmptyInstallations(message = null)
    {
        const tabsEl = document.getElementById('installation-tabs');
        const contentEl = document.getElementById('installation-content');

        if (tabsEl)
        {
            tabsEl.innerHTML = `
        <div class="empty-state">
          <div class="empty-state-icon">🔍</div>
          <h3>${message || 'No Installations Found'}</h3>
          <p>${message ? '' : 'Configure your SC directory in Auto Save Settings to detect installations'}</p>
        </div>
      `;
        }

        if (contentEl)
        {
            contentEl.innerHTML = '';
        }
    }

    switchInstallation(installName)
    {
        this.activeInstallation = installName;

        // Update active tab
        document.querySelectorAll('.installation-tab').forEach(tab =>
        {
            if (tab.dataset.install === installName)
            {
                tab.classList.add('active');
            } else
            {
                tab.classList.remove('active');
            }
        });

        // Render installation content
        this.renderInstallationContent(installName);
    }

    renderInstallationContent(installName)
    {
        const contentEl = document.getElementById('installation-content');
        if (!contentEl) return;

        const installation = this.installations.find(i => i.name === installName);
        if (!installation) return;

        const characters = this.installationCharacters[installName] || [];

        contentEl.innerHTML = `
      <div class="installation-panel active">
        <div class="installation-header">
          <div>
            <h4 style="margin: 0 0 0.25rem 0;">${installName} Installation</h4>
            <div class="installation-path">${installation.path}</div>
          </div>
          <button class="btn btn-primary" onclick="characterManager.deployAllToInstallation('${installName}')">
            📥 Import All from Library
          </button>
        </div>

        ${characters.length === 0 ? `
          <div class="empty-state">
            <div class="empty-state-icon">📭</div>
            <h3>No Characters Found</h3>
            <p>No character files in this installation</p>
          </div>
        ` : `
          <div class="installation-characters-list">
            ${characters.map(char => this.renderInstallationCharacter(char, installName)).join('')}
          </div>
        `}
      </div>
    `;
    }

    renderInstallationCharacter(char, installName)
    {
        const masterChar = this.masterCharacters.find(m => m.name === char.name);

        let status = 'missing';
        let statusText = 'Not in Library';

        if (masterChar)
        {
            if (char.modified === masterChar.modified)
            {
                status = 'up-to-date';
                statusText = 'Up to Date';
            } else if (char.modified > masterChar.modified)
            {
                status = 'newer';
                statusText = 'Newer';
            } else
            {
                status = 'outdated';
                statusText = 'Outdated';
            }
        }

        return `
      <div class="character-card">
        <div class="character-icon">👤</div>
        <div class="character-info">
          <h4 class="character-name">${char.name}</h4>
          <div class="character-meta">
            <span>📅 ${this.formatDate(char.modified)}</span>
            <span>💾 ${this.formatFileSize(char.size)}</span>
            <span class="character-status ${status}">${statusText}</span>
          </div>
        </div>
        <div class="character-actions">
          ${status === 'newer' || status === 'missing' ? `
            <button class="btn btn-primary btn-sm" onclick="characterManager.importToLibrary('${char.name}', '${installName}')" title="Import to library">
              📥 Import
            </button>
          ` : ''}
          ${masterChar ? `
            <button class="btn btn-secondary btn-sm" onclick="characterManager.deployToInstallation('${char.name}', '${installName}')" title="Deploy from library">
              📤 Deploy
            </button>
          ` : ''}
          <button class="btn btn-danger btn-sm" onclick="characterManager.deleteFromInstallation('${char.name}', '${installName}')" title="Delete from installation">
            🗑️ Delete
          </button>
        </div>
      </div>
    `;
    }

    async deployToInstallation(characterName, installName)
    {
        if (!this.libraryPath)
        {
            this.showError('Library path not configured');
            return;
        }

        const installation = this.installations.find(i => i.name === installName);
        if (!installation)
        {
            this.showError('Installation not found');
            return;
        }

        try
        {
            await invoke('deploy_character_to_installation', {
                characterName,
                libraryPath: this.libraryPath,
                installationPath: installation.path
            });

            this.showSuccess(`Deployed ${characterName} to ${installName}`);
            await this.loadInstallationCharacters(installation);
            this.renderInstallationContent(installName);
        } catch (error)
        {
            console.error('Error deploying character:', error);
            this.showError('Failed to deploy character: ' + error);
        }
    }

    async deployAllToInstallation(installName)
    {
        if (!this.libraryPath || this.masterCharacters.length === 0)
        {
            this.showError('No characters in library to deploy');
            return;
        }

        const installation = this.installations.find(i => i.name === installName);
        if (!installation)
        {
            this.showError('Installation not found');
            return;
        }

        try
        {
            for (const char of this.masterCharacters)
            {
                await invoke('deploy_character_to_installation', {
                    characterName: char.name,
                    libraryPath: this.libraryPath,
                    installationPath: installation.path
                });
            }

            this.showSuccess(`Deployed all characters to ${installName}`);
            await this.loadInstallationCharacters(installation);
            this.renderInstallationContent(installName);
        } catch (error)
        {
            console.error('Error deploying all characters:', error);
            this.showError('Failed to deploy all characters: ' + error);
        }
    }

    async importToLibrary(characterName, installName)
    {
        if (!this.libraryPath)
        {
            this.showError('Library path not configured');
            return;
        }

        const installation = this.installations.find(i => i.name === installName);
        if (!installation)
        {
            this.showError('Installation not found');
            return;
        }

        try
        {
            await invoke('import_character_to_library', {
                characterName,
                installationPath: installation.path,
                libraryPath: this.libraryPath
            });

            this.showSuccess(`Imported ${characterName} to library`);
            await this.loadMasterCharacters();
            await this.loadInstallationCharacters(installation);
            this.renderInstallationContent(installName);
        } catch (error)
        {
            console.error('Error importing character:', error);
            this.showError('Failed to import character: ' + error);
        }
    }

    async exportCharacter(characterName)
    {
        if (!this.libraryPath)
        {
            this.showError('Library path not configured');
            return;
        }

        if (this.installations.length === 0)
        {
            this.showError('No installations found');
            return;
        }

        try
        {
            for (const installation of this.installations)
            {
                await invoke('deploy_character_to_installation', {
                    characterName,
                    libraryPath: this.libraryPath,
                    installationPath: installation.path
                });
            }

            this.showSuccess(`Deployed ${characterName} to all installations`);

            // Reload all installation characters
            for (const installation of this.installations)
            {
                await this.loadInstallationCharacters(installation);
            }

            // Re-render master characters to update sync status
            this.renderMasterCharacters();

            if (this.activeInstallation)
            {
                this.renderInstallationContent(this.activeInstallation);
            }
        } catch (error)
        {
            console.error('Error exporting character:', error);
            this.showError('Failed to export character: ' + error);
        }
    }

    async deleteCharacter(characterName)
    {
        if (!this.libraryPath)
        {
            this.showError('Library path not configured');
            return;
        }

        // Use showConfirmation from main.js (available globally via window)
        const showConfirmation = window.showConfirmation;
        if (!showConfirmation)
        {
            this.showError('Confirmation dialog not available');
            return;
        }

        const confirmed = await showConfirmation(
            `Delete "${characterName}" from your library?\n\nThis will NOT delete it from your game installations.`,
            'Delete Character',
            'Delete',
            'Cancel',
            'btn-danger'
        );

        if (!confirmed)
        {
            return;
        }

        try
        {
            await invoke('delete_character_from_library', {
                characterName,
                libraryPath: this.libraryPath
            });

            this.showSuccess(`Deleted ${characterName} from library`);
            await this.loadMasterCharacters();
        } catch (error)
        {
            console.error('Error deleting character:', error);
            this.showError('Failed to delete character: ' + error);
        }
    }

    async deleteFromInstallation(characterName, installName)
    {
        const installation = this.installations.find(i => i.name === installName);
        if (!installation)
        {
            this.showError('Installation not found');
            return;
        }

        // Use showConfirmation from main.js (available globally via window)
        const showConfirmation = window.showConfirmation;
        if (!showConfirmation)
        {
            this.showError('Confirmation dialog not available');
            return;
        }

        const confirmed = await showConfirmation(
            `Delete "${characterName}" from ${installName}?`,
            'Delete Character',
            'Delete',
            'Cancel',
            'btn-danger'
        );

        if (!confirmed)
        {
            return;
        }

        try
        {
            await invoke('delete_character_from_installation', {
                installationPath: installation.path,
                characterName
            });

            this.showSuccess(`Character "${characterName}" deleted from ${installName}`);
            await this.loadInstallationCharacters(installName);
            await this.renderMasterCharacters(); // Update sync status
        } catch (error)
        {
            console.error('Error deleting character from installation:', error);
            this.showError(`Error deleting character: ${error}`);
        }
    }

    formatDate(timestamp)
    {
        const date = new Date(timestamp);
        return date.toLocaleDateString() + ' ' + date.toLocaleTimeString();
    }

    formatFileSize(bytes)
    {
        if (bytes < 1024) return bytes + ' B';
        if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB';
        return (bytes / (1024 * 1024)).toFixed(1) + ' MB';
    }

    showSuccess(message)
    {
        this.showNotification(message, 'success');
    }

    showError(message)
    {
        this.showNotification(message, 'error');
    }

    showNotification(message, type = 'info')
    {
        // Create notification element
        const notification = document.createElement('div');
        notification.className = `character-notification character-notification-${type}`;

        const icon = type === 'success' ? '✅' : type === 'error' ? '❌' : 'ℹ️';
        notification.innerHTML = `
            <span class="notification-icon">${icon}</span>
            <span class="notification-message">${message}</span>
            <button class="notification-close" onclick="this.parentElement.remove()">×</button>
        `;

        // Add to page
        document.body.appendChild(notification);

        // Auto-remove after 5 seconds
        setTimeout(() =>
        {
            if (notification.parentElement)
            {
                notification.remove();
            }
        }, 5000);
    }
}

// Initialize character manager when on the character tab
let characterManager = null;

// Export initialization function for main.js to call
window.initCharacterManager = function ()
{
    if (!characterManager)
    {
        characterManager = new CharacterManager();
        window.characterManager = characterManager;
    }
};

// Listen for tab changes
document.addEventListener('DOMContentLoaded', () =>
{
    const characterTab = document.getElementById('tab-character');

    if (characterTab)
    {
        characterTab.addEventListener('click', () =>
        {
            window.initCharacterManager();
        });
    }
});
