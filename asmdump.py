Uniquement pour le project atmelavr

Import("env", "projenv")
# Ajout de l'option -g aux indicateurs du linker pour obtenir le code source dans le fichier elf.
env.Append(LINKFLAGS=["-g"])

# Ajout d'un outil dans le menu Custom de PIO
env.AddCustomTarget(
    "asm_dump",
    "$BUILD_DIR/${PROGNAME}.elf",
    "/Users/dupuyb/.platformio/packages/toolchain-atmelavr/avr/bin/objdump -D -S '$BUILD_DIR/${PROGNAME}.elf' > '$BUILD_DIR/${PROGNAME}.asm'",
    "Dump ELF to ASM",
    "Dump de firmware.elf vers firmware.asm (source + ASM résultant de la compilation)."
)

# Fonction de call back pour extraire le code asm compilé avec le code source
def asmAvecSourceCB(source, target, env):
    print("Dump firmware.elf ...")
    #env.Execute("echo BUILD_DIR=$BUILD_DIR")
    env.Execute("/Users/dupuyb/.platformio/packages/toolchain-atmelavr/avr/bin/objdump -D -S '$BUILD_DIR/${PROGNAME}.elf' > '$BUILD_DIR/${PROGNAME}.asm'")
    # do some actions

# Ajout d'une fonction de call back a exécuté lorsque le fichier firmware.elf est créé.
env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", asmAvecSourceCB)