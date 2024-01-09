# 122-2024

# Intellisense

Install clangd vscode extension. Run the following gradle command at the root

```
./gradlew generateCompileCommands
```

If the clangd extension doesn't detect the compile commands file, copy compile_commands.json out of the build folder into the project root.

# Code formatting

Code formatting is done with the wpiformat automatic formatter, run the following at the root

```
python3 -m pip install wpiformat
python3 -m wpiformat
```