msdos mcc68k m68cbios.c m68kbios.src null.lst
del null.lst
sed "s/\tSECTION/*\tSECTION/g" m68kbios.src > aaa
sed "s/\tXREF/*\tXREF/g" aaa > bbb
sed "s/\tOPT/*\tOPT/g" bbb > m68kbios.src
