# ThinkOO

A VSCode extension.

## Why doesn't VSCode update immediately after upgrading?

When you upgrade the ThinkOO extension in VSCode, the new version is installed but
the running instance still uses the old code. VSCode must reload the window to
activate the updated extension.

Starting from this version the extension automatically detects when it has been
upgraded. A notification appears at the bottom-right of the window:

> **ThinkOO has been updated to vX.Y.Z. Please reload the window for the changes to take effect.**
> `Reload Now` | `Later`

Click **Reload Now** to restart the extension host and apply the update immediately.
If you choose **Later**, the update will take effect the next time you restart VSCode.

## Development

```bash
npm install
npm run compile
```

Open the folder in VSCode and press <kbd>F5</kbd> to launch the Extension Development Host.
