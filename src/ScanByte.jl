module ScanByte

using Libdl
using SIMD

include("byteset.jl")
include("codegen.jl")

export SizedMemory, ByteSet, gen_scan_function, memchr

end # module