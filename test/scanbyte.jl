@testset "SizedMemory" begin
    for str in ["Hello, world!", "", "αβγ", "    "]
        mem = SizedMemory(str)
        @test pointer(mem) == pointer(str)
        @test length(mem) == ncodeunits(str)

        arr = collect(codeunits(str))
        mem = SizedMemory(arr)
        @test pointer(mem) == pointer(arr)
        @test length(mem) == length(arr)
    end

    for str in ["Hello, world!", "αβγδ", "          "]
        for start in (1, 3), len in (0, 3, 5)
            substr = SubString(str, start:start + len - 1)
            mem = SizedMemory(substr)
            @test pointer(mem) == pointer(substr)
            @test length(mem) == ncodeunits(substr)
        end
    end
end

@testset "memchr" begin
    for T in (nothing, ScanByte.v128, ScanByte.v256)
        for byte in [0x00, 0xa0, 0xda, 0xff]
            @test ScanByte._memchr(T, SizedMemory(UInt8[]), byte) === nothing
            not_bytes = [i for i in 0x00:0xff if i != byte]
            for (len, ind) in [(2, 2), (20, 14), (500, 431)]
                v = rand(not_bytes, len)
                @test ScanByte._memchr(T, SizedMemory(v), byte) === nothing
                v[ind] = byte
                @test ScanByte._memchr(T, SizedMemory(v), byte) == ind
            end
        end
    end

    # Also test memchr directly (instead of _memchr)
    @test memchr(SizedMemory([]), 0x01) === nothing
    @test memchr(SizedMemory([1,2,3]), 0x02) === 9
    @test memchr(SizedMemory([1,2,3]), 0x04) === nothing
end
            

@testset "Scanning" begin
    # We test the inverted bytesets, because the codegen is built to look for
    # the bytes its NOT scanning for. Just to make the test easier
    for T in (ScanByte.v128, ScanByte.v256, nothing)
        for inv_byteset in [
            ByteSet(), # empty
            ByteSet(0x00:0xff), # full
            ByteSet(0x41), # one member
            ~ByteSet(0x6a), # all but one member
            ByteSet([0xa2, 0xb4, 0xc8, 0xf9, 0xf6]), # unique lower nibble
            ~ByteSet([0x38, 0x40, 0x51, 0x79, 0x94]), # inv uniq nibble
            ByteSet(0x49:0x71), # Contiguous
            ~ByteSet(0x02:0x31), # Inverted contiguous
            ByteSet(rand(0x80:0xff, 20)), # 128 - 255
            ByteSet(rand(0x00:0x7f, 20)), # 0 - 127
            ByteSet(rand(0x5a:0x94, 20)), # within 127 of each other
            ~ByteSet(rand(0x5a:0x94, 20)), # inv within 127
            ByteSet([0x38, 0x40, 0x90, 0xba, 0xc5, 0xc8, 0xe7]), # at most 8 elements
            ByteSet([i for i in 0x00:0xff if rand(Bool)]) # fallback
        ]
            byteset = ~inv_byteset
            eval(ScanByte._gen_scan_function(T, :scanbyte, byteset))

            # Empty
            @test scanbyte(SizedMemory(UInt8[])) === nothing

            # 1000 bytes not in the set
            if length(byteset) != 256
                bytes = rand(collect(~byteset), 1000)
                @test scanbyte(SizedMemory(bytes)) === nothing

                if !isempty(byteset)
                    bytes[500] = first(iterate(byteset))
                    @test scanbyte(SizedMemory(bytes)) == 500

                    bytes[25] = first(iterate(byteset))
                    @test scanbyte(SizedMemory(bytes)) == 25
                end

                resize!(bytes, 20)
                @test scanbyte(SizedMemory(bytes)) === nothing
            end

            # Short vector of bytes in the set
            if !isempty(byteset)
                bytes = rand(collect(byteset), 128)
                @test scanbyte(SizedMemory(bytes)) == 1
            end
        end
    end

    # Also test gen_scan_function directly (instead of _gen_scan_function)
    eval(ScanByte.gen_scan_function(:scanbyte, ByteSet([0x02, 0x09, 0x11])))
    @test scanbyte(SizedMemory([1,2,3])) == 9
    @test scanbyte(SizedMemory([1,3,4])) === nothing
    @test scanbyte(SizedMemory([1,5,6,7,4,17,13])) == 41
end
