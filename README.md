# TinyPICbootloader
Serial bootloader for PIC 18F and program uploader for Linux. Both used in the Djuke Preamplifier project.

# PIC bootloader
Based on the work of Claudiu Chiculita. More information on:

[Tiny PIC bootloader](http://www.etc.ugal.ro/cchiculita/software/picbootloader.htm)

## Usage
Requires gputils

```
cd PIC18F
gpasm <filename>.asm -p PIC18F4620
```

# Program uploader
Based on the work of José Antonio Robles Ordóñez

## Usage
Requires libtclap-dev for command-line parsing

```
cd UnitTinyBootLoaderLoader
make
```

