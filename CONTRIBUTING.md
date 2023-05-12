Contributing
============

Setup Environment
-----------------
Install the following software

- [Visual Studio Code](https://code.visualstudio.com/)
- [Git](https://git-scm.com/)
- Clang-Format (Bundled with 'Clang')
- [PROS-CLI](https://pros.cs.purdue.edu/v5/index.html) *(Optional)*

You will also need the following VSCode extensions:

- [PROS-VSC](https://marketplace.visualstudio.com/items?itemName=sigbots.pros)
- [Clang-Format](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format) (Requires Clang-Format to be installed seperately)
- [Clangd](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd) (Disable microsoft C/C++)

After installing the necessary software, [set up git on your machine](https://git-scm.com/book/en/v2/Getting-Started-First-Time-Git-Setup) and clone this project.

From there, in the project directory, create a file at `.vscode/settings.json` with (at least) the following entries:

```json
{
	"editor.formatOnSave": true,
    "editor.detectIndentation": false,
    "editor.insertSpaces": false,
    "editor.defaultFormatter": "xaver.clang-format",
	"clang-format.style": "file",
}
```
