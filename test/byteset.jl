function test_membership(members)
    bs = ByteSet(members)
    refset = Set{UInt8}([UInt8(i) for i in  members])
    @test refset == Set{UInt8}(collect(bs))
    @test all(i -> in(i, bs), refset)
end

function test_inversion(bs)
    inv = ~bs
    all = true
    for i in 0x00:0xff
        all &= (in(i, bs) ⊻ in(i, inv))
    end
    @test all
end

@testset "Instantiation" begin
    @test isempty(ByteSet())
    @test iszero(length(ByteSet()))

    for set in ["hello", "kdjy82zxxcbnpw", [0x00, 0x1a, 0xff, 0xf8, 0xd2]]
        test_membership(set)
    end
end

@testset "Min/max" begin
    @test_throws ArgumentError maximum(ByteSet())
    @test_throws ArgumentError minimum(ByteSet())
    @test minimum(ByteSet("xylophone")) == UInt8('e')
    @test maximum(ByteSet([0xa1, 0x0f, 0x4e, 0xf1, 0x40, 0x39])) == 0xf1
end

@testset "Contiguity" begin
    @test ScanByte.is_contiguous(ByteSet(0x03:0x41))
    @test ScanByte.is_contiguous(ByteSet())
    @test ScanByte.is_contiguous(ByteSet(0x51))
    @test ScanByte.is_contiguous(ByteSet(0xc1:0xd2))
    @test ScanByte.is_contiguous(ByteSet(0x00:0xff))

    @test !ScanByte.is_contiguous(ByteSet([0x12:0x3a; 0x3c:0x4a]))
    @test !ScanByte.is_contiguous(ByteSet([0x01, 0x02, 0x04, 0x05]))
end

@testset "Inversion" begin
    test_inversion(ByteSet())
    test_inversion(ByteSet(0x00:0xff))
    test_inversion(ByteSet([0x04, 0x06, 0x91, 0x92]))
    test_inversion(ByteSet(0x54:0x71))
    test_inversion(ByteSet(0x12:0x11))
    test_inversion(ByteSet("abracadabra"))
end