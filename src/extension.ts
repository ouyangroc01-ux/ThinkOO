import * as vscode from 'vscode';

const LAST_VERSION_KEY = 'thinkoo.lastVersion';

export function activate(context: vscode.ExtensionContext) {
    const currentVersion: string = context.extension.packageJSON.version;
    const lastVersion = context.globalState.get<string>(LAST_VERSION_KEY);

    if (lastVersion !== currentVersion) {
        if (lastVersion !== undefined) {
            // This is an upgrade, not the first install — prompt the user to reload
            vscode.window
                .showInformationMessage(
                    `ThinkOO has been updated to v${currentVersion}. Please reload the window for the changes to take effect.`,
                    'Reload Now',
                    'Later'
                )
                .then((selection) => {
                    if (selection === 'Reload Now') {
                        vscode.commands.executeCommand('workbench.action.reloadWindow');
                    }
                });
        }
        // Persist the current version so subsequent activations skip this check
        context.globalState.update(LAST_VERSION_KEY, currentVersion);
    }
}

export function deactivate() {}
