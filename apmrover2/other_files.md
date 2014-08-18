
#Makefile

Here you can find the link to the code: [Makefile](https://github.com/BeaglePilot/ardupilot/blob/master/APMrover2/Makefile).

---

######Makefile and make utility

Makefiles are special format files that together with the make utility will help you to automatically build and manage your projects.

If you run on your terminal window:
```
make
```
this utility will look for a file named "makefile" in your directory, and then execute it.
If you have several makefiles, then you can execute them with the command:
```
make -f MyMakefile
```
For more info type: `man make`.

---

The Makefile includes the `apm.k`:
```
include ../mk/apm.mk
```
and let you build the ardurover.
