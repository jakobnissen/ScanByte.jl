module ScanByte

using Libdl: Libdl
using SIMD: SIMD

include("byteset.jl")
include("codegen.jl")

function __init__()
    if DEFVEC === nothing
        @warn "SIMD capacity not detected by ScanByte, using scalar fallback"
    end
end

export ByteSet, memchr

end # module
