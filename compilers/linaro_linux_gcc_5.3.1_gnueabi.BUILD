package(default_visibility = ['//visibility:public'])

filegroup(
  name = 'gcc',
  srcs = [
    'bin/arm-linux-gnueabi-gcc',
  ],
)

filegroup(
  name = 'ar',
  srcs = [
    'bin/arm-linux-gnueabi-ar',
  ],
)

filegroup(
  name = 'ld',
  srcs = [
    'bin/arm-linux-gnueabi-ld',
  ],
)

filegroup(
  name = 'nm',
  srcs = [
    'bin/arm-linux-gnueabi-nm',
  ],
)

filegroup(
  name = 'objcopy',
  srcs = [
    'bin/arm-linux-gnueabi-objcopy',
  ],
)

filegroup(
  name = 'objdump',
  srcs = [
    'bin/arm-linux-gnueabi-objdump',
  ],
)

filegroup(
  name = 'strip',
  srcs = [
    'bin/arm-linux-gnueabi-strip',
  ],
)

filegroup(
  name = 'as',
  srcs = [
    'bin/arm-linux-gnueabi-as',
  ],
)

filegroup(
  name = 'compiler_pieces',
  srcs = glob([
    'arm-linux-gnueabi/**',
    'libexec/**',
    'lib/gcc/arm-linux-gnueabi/**',
    'include/**',
  ]),
)

filegroup(
  name = 'compiler_components',
  srcs = [
    ':gcc',
    ':ar',
    ':ld',
    ':nm',
    ':objcopy',
    ':objdump',
    ':strip',
    ':as',
  ],
)
