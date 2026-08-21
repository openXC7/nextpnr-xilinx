/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  OCaml 5.x Random, reproduced exactly.
 *
 *  WHY THIS EXISTS.  place_lef.exe is the placer every silicon-verified
 *  bitstream on this project went through, and porting it into nextpnr has
 *  repeatedly foundered on the question "is my version doing the same thing?".
 *  Aggregate metrics cannot answer that: a port can score better on net
 *  locality, pin demand, area demand and carry co-location and still route
 *  far worse, which is exactly what happened.
 *
 *  Every decision place_lef's annealer makes is drawn from OCaml's Random --
 *  which cell to move, which site to try, whether to accept:
 *
 *      let i = mv.(Random.int m)
 *      let s = pool.(Random.int (Array.length pool))
 *      let accept = delta <= 0.0 || Random.float 1.0 < exp (-. delta /. !t)
 *
 *  So with the same seed and the same PRNG, a faithful transliteration must
 *  produce the IDENTICAL placement, and `make placement-ab` reports 100% or it
 *  does not.  That turns "did I capture the algorithm?" into a test.
 *
 *  OCaml 5 uses LXM (Steele & Vigna): an LCG combined with xoroshiro128,
 *  finalised through the MurmurHash3 mixer.  Seeding hashes the seed array
 *  with MD5 twice, under 0x01 and 0x02 suffixes, to fill 256 bits of state
 *  (stdlib/random.ml: reinit).
 *
 *  VERIFIED against ocaml 5.3.0 for seeds 1 and 7919: the first eight
 *  Random.int 1000000 and the first four Random.float 1.0 match bit-for-bit.
 *  self_test() below carries those vectors; call it before trusting a port.
 */

#ifndef OCAML_RANDOM_H
#define OCAML_RANDOM_H

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace ocaml_random {

// --- MD5, needed only for seeding (stdlib/random.ml hashes the seed array) ---
struct MD5
{
    uint32_t a0 = 0x67452301, b0 = 0xefcdab89, c0 = 0x98badcfe, d0 = 0x10325476;
    static uint32_t rotl32(uint32_t x, int c) { return (x << c) | (x >> (32 - c)); }
    // One-shot: messages here are a handful of bytes, so no streaming needed.
    void digest(const uint8_t *msg, size_t len, uint8_t out[16])
    {
        static const uint32_t K[64] = {
                0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
                0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
                0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
                0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
                0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
                0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
                0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
                0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391};
        static const int S[64] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22,
                                  5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20,
                                  4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23,
                                  6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21};
        std::vector<uint8_t> m(msg, msg + len);
        m.push_back(0x80);
        while (m.size() % 64 != 56)
            m.push_back(0);
        uint64_t bits = uint64_t(len) * 8;
        for (int i = 0; i < 8; i++)
            m.push_back(uint8_t(bits >> (8 * i)));
        for (size_t off = 0; off < m.size(); off += 64) {
            uint32_t M[16];
            for (int i = 0; i < 16; i++)
                std::memcpy(&M[i], &m[off + i * 4], 4);
            uint32_t A = a0, B = b0, C = c0, D = d0;
            for (int i = 0; i < 64; i++) {
                uint32_t F;
                int g;
                if (i < 16) { F = (B & C) | (~B & D); g = i; }
                else if (i < 32) { F = (D & B) | (~D & C); g = (5 * i + 1) % 16; }
                else if (i < 48) { F = B ^ C ^ D; g = (3 * i + 5) % 16; }
                else { F = C ^ (B | ~D); g = (7 * i) % 16; }
                F = F + A + K[i] + M[g];
                A = D; D = C; C = B;
                B = B + rotl32(F, S[i]);
            }
            a0 += A; b0 += B; c0 += C; d0 += D;
        }
        uint32_t h[4] = {a0, b0, c0, d0};
        for (int i = 0; i < 4; i++)
            std::memcpy(out + i * 4, &h[i], 4);
    }
};

struct State
{
    uint64_t v[4]; // v[0] additive (odd), v[1] LCG, v[2..3] xoroshiro128

    static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

    // stdlib/random.ml: reinit -- serialise the seed array little-endian, append
    // 0x01 and MD5 it, then 0x02 and MD5 again, to fill 256 bits.
    void init(const std::vector<int64_t> &seed)
    {
        size_t n = seed.size();
        std::vector<uint8_t> b(n * 8 + 1, 0);
        for (size_t i = 0; i < n; i++) {
            uint64_t s = uint64_t(seed[i]);
            for (int k = 0; k < 8; k++)
                b[i * 8 + k] = uint8_t(s >> (8 * k));
        }
        uint8_t d1[16], d2[16];
        b[n * 8] = 1;
        MD5().digest(b.data(), b.size(), d1);
        b[n * 8] = 2;
        MD5().digest(b.data(), b.size(), d2);
        auto le64 = [](const uint8_t *p) {
            uint64_t r = 0;
            for (int i = 7; i >= 0; i--)
                r = (r << 8) | p[i];
            return r;
        };
        v[0] = le64(d1) | 1ULL;                            // must be odd
        v[1] = le64(d1 + 8);
        v[2] = le64(d2) ? le64(d2) : 1ULL;                 // must not be 0
        v[3] = le64(d2 + 8) ? le64(d2 + 8) : 2ULL;         // must not be 0
    }

    void init(int64_t seed) { init(std::vector<int64_t>{seed}); }

    uint64_t next()
    {
        const uint64_t MULT = 0xd1342543de82ef95ULL, MIX = 0xdaba0b6eb09322e3ULL;
        uint64_t a = v[0], s = v[1], x0 = v[2], x1 = v[3];
        uint64_t z = s + x0;
        z = (z ^ (z >> 32)) * MIX;
        z = (z ^ (z >> 32)) * MIX;
        z = z ^ (z >> 32);
        v[1] = s * MULT + a;              // LCG
        uint64_t q0 = x0, q1 = x1;        // xoroshiro128
        q1 ^= q0;
        q0 = rotl(q0, 24);
        q0 = q0 ^ q1 ^ (q1 << 16);
        q1 = rotl(q1, 37);
        v[2] = q0;
        v[3] = q1;
        return z;
    }

    // Int64.to_int drops to 63 bits, signed; `int` then masks to 30 bits and
    // rejection-samples so the result is uniform (stdlib/random.ml: int_aux).
    int64_t to_int(uint64_t z) const
    {
        int64_t sv = int64_t(z);
        return (sv << 1) >> 1;
    }

    int int_(int bound)
    {
        const int64_t mask = 0x3FFFFFFF;
        for (;;) {
            int64_t r = to_int(next()) & mask;
            int64_t vv = r % bound;
            if (r - vv <= mask - bound + 1)
                return int(vv);
        }
    }

    // rawfloat: (next >> 11) * 2^-53, redrawn if zero
    double float_(double bound)
    {
        for (;;) {
            uint64_t n = next() >> 11;
            if (n != 0)
                return double(n) * 0x1p-53 * bound;
        }
    }
};

// Vectors taken from ocaml 5.3.0 directly.  A port that fails this is not
// reproducing place_lef's decisions and any comparison against it is void.
inline bool self_test()
{
    static const int want1[8] = {492614, 250914, 676862, 544251, 161068, 178514, 522891, 88776};
    static const int want2[4] = {88725, 569350, 101238, 892627};
    static const double wantf[4] = {0.32954457993056696, 0.28951489086551863, 0.60541793926797771,
                                    0.36351037263890573};
    State s;
    s.init(1);
    for (int i = 0; i < 8; i++)
        if (s.int_(1000000) != want1[i])
            return false;
    s.init(1);
    for (int i = 0; i < 4; i++)
        if (s.float_(1.0) != wantf[i])
            return false;
    s.init(7919);
    for (int i = 0; i < 4; i++)
        if (s.int_(1000000) != want2[i])
            return false;
    return true;
}

} // namespace ocaml_random

#endif
