To install the pic32 tools (Linux only, for now):
  1) make sure you have the tarball:
     - pic32-32.tgz for 32-bit systems (or 64-bit systems with 32- and 64-bit libraries)
     - pic32-64.tgz for 64-bit systems
  2) execute the command
       sudo tar xzvf TARBALL -C /opt
     where TARBALL is the full path of the tarball; for example, if the tarball is
     /tmp/pic32-32.tgz, the command should be
       sudo tar xzvf /tmp/pic32-32.tgz -C /opt
     you will need administration privileges in order to do this
  3) add the lines
       if [ -d /opt/pic32mx/bin ] ; then
         export PATH=$PATH:/opt/pic32mx/bin
       fi
     at the end of your .bashrc and your .profile files
     (i.e., of one of the files ~/.bashrc or ~/.profile)

To uninstall the pic32 tools execute the command
  sudo rm -rf /opt/pic32mx
you will need administration privileges in order to do this


The program
  pcompile
should be used to compile a program. It works with C (*.c) and ASM (*.s) source files.
To compile just do
  pcompile source_file_name
This will produce a HEX file (ready to be downloaded to the DETPIC32 kit), a MAP file,
a ELF file, and and OBJECT file. Only the HEX file is trully needed. The name of the
HEX file is the name of the first source file (with .c or .s replaced by .hex).
To remove all generated files do
  pcompile
without any arguments.

To transfer a HEX program to the DETPIC32 kit use the command ldpic32 as follows:
  ldpic32 -w file_name.hex
where file_name.hex should be replaced by the name of your HEX file. To get some
information about what is loaded in the DETPIC32 kit use the command
  ldpic32 -i
The HEX file can be disassembled with the command
  hex2asm file_name.hex
By default, the disassembled code will be written in a file named file_name.hex.s
