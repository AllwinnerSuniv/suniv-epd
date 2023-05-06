<h1 align="center"> Suniv driver template </h1>

This a linux driver template for Allwinner suniv series SoC.

# Build

modify the `KERN_DIR` in  `Makefile` to your real kernel source or just export it.

then

```bash
bear -- make
```

edit in vscode

```bash
code .
```

make sure clangd was configured correctly.

# Depoly

In device uart though minicom /dev/ttyUSB0 etc.

```bash
rz
```

type `Ctrl+A S` choose `zmodem` and filled the input with file path

