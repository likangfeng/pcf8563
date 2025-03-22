from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add pcf8563 src files.
if GetDepend('PKG_USING_PCF8563'):
    src += Glob('pcf8563.c')

# add pcf8563 include path.
path  = [cwd]

# add src and include to group.
group = DefineGroup('pcf8563', src, depend = ['PKG_USING_PCF8563'], CPPPATH = path)

Return('group')
