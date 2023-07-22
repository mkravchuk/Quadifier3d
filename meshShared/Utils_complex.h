#pragma once


namespace utils
{
    namespace complex
    {
        const float sqrt2f = std::sqrt(0.5f);
        const __m128 sqrt22f{ sqrt2f, sqrt2f };
        const __m128 sqrt2222f{ sqrt2f, sqrt2f, sqrt2f, sqrt2f };
        const double sqrt2 = std::sqrt(0.5);
        const __m128d sqrt22{ sqrt2, sqrt2 };
        const __m128 sqrt2222{ sqrt2f, sqrt2f,sqrt2f,sqrt2f };

        __forceinline float absPow2(std::complex<float> a)
        { 
            float r = a.real();
            float i = a.imag();
            return r * r + i * i;
        }
        __forceinline double absPow2(std::complex<double> a)
        {
            double r = a.real();
            double i = a.imag();
            return r * r + i * i;
        }
        __forceinline float abs(std::complex<float> a)
        {
            return std::sqrt(absPow2(a));
        }
        __forceinline double abs(std::complex<double> a)
        {
            return std::sqrt(absPow2(a));
        }
        __forceinline std::complex<float>  sqrt(std::complex<float> a)
        {
            float r = a.real();
            float i = a.imag();

            float det = std::sqrt(r*r + i * i);
            float r2 = sqrt2f * std::sqrt(r + det);
            float i2 = sqrt2f * std::sqrt(-r + det);
            if (i < 0) i2 = -i2;
            //cout << sqrt(a) << "    " << r2 <<","<<i2 << endl;
            return std::complex<float>(r2, i2);
        }
        __forceinline std::complex<double>  sqrt(std::complex<double> a)
        {
            double r = a.real();
            double i = a.imag();

            double det = std::sqrt(r*r + i * i);
            double r2 = sqrt2 * std::sqrt(r + det);
            double i2 = sqrt2 * std::sqrt(-r + det);
            if (i < 0) i2 = -i2;
            //cout << sqrt(a) << "    " << r2 <<","<<i2 << endl;
            return std::complex<double>(r2, i2);
        }

        __forceinline std::complex<float>  sqrtFast(std::complex<float> a)
        {
            float r = a.real();
            float i = a.imag();

            //v0
            //double det = std::sqrt(r*r + i * i);
            //double r2 = sqrt2 * std::sqrt(r + det);
            //double i2 = sqrt2 * std::sqrt(-r + det);
            //v1                         
            float det = _mm_sqrt_ps({ r*r + i * i,0 }).m128_f32[0];
            __m128 s{ r + det, -r + det };
            __m128 sr = _mm_mul_ps(_mm_sqrt_ps(s), sqrt22f);
            //__m128d sr = _mm_sqrt_pd(s);
            float r2 = sr.m128_f32[0];
            float i2 = sr.m128_f32[1];
            //cout << "r2-r2 = " << r2 - sr.m128d_f64[0] << "      i2-i2 = " << i2 - sr.m128d_f64[1] << endl;
            if (i < 0) i2 = -i2;
            //cout << sqrt(a) << "    " << r2 <<","<<i2 << endl;
            return std::complex<float>(r2, i2);
        }
        __forceinline std::complex<double>  sqrtFast(std::complex<double> a)
        {
            double r = a.real();
            double i = a.imag();

            //v0
            //double det = std::sqrt(r*r + i * i);
            //double r2 = sqrt2 * std::sqrt(r + det);
            //double i2 = sqrt2 * std::sqrt(-r + det);
            //v1                         
            double det = _mm_sqrt_pd({ r*r + i * i,0 }).m128d_f64[0];
            __m128d s{ r + det, -r + det };
            __m128d sr = _mm_mul_pd(_mm_sqrt_pd(s), sqrt22);
            //__m128d sr = _mm_sqrt_pd(s);
            double r2 = sr.m128d_f64[0];
            double i2 = sr.m128d_f64[1];
            //cout << "r2-r2 = " << r2 - sr.m128d_f64[0] << "      i2-i2 = " << i2 - sr.m128d_f64[1] << endl;
            if (i < 0) i2 = -i2;
            //cout << sqrt(a) << "    " << r2 <<","<<i2 << endl;
            return std::complex<double>(r2, i2);
        }

        __forceinline void  sqrtFast2(const std::complex<float>& a, const std::complex<float>& b, std::complex<float>& asqrt, std::complex<float>& bsqrt)
        {
            const float& ar = a.real();
            const float& ai = a.imag();
            const float& br = b.real();
            const float& bi = b.imag();

            //_mm_csqrt_ps({ (float)ar, (float)ai, (float)br, (float)bi });
            //v0
            //double det = std::sqrt(r*r + i * i);
            //double r2 = sqrt2 * std::sqrt(r + det);
            //double i2 = sqrt2 * std::sqrt(-r + det);
            //v1 
            __m128 det = _mm_sqrt_ps({ ar*ar + ai * ai,br*br + bi * bi });
            float deta = det.m128_f32[0];
            float detb = det.m128_f32[1];
            __m128 s{ ar + deta, -ar + deta,  br + detb, -br + detb };
            __m128 sr = _mm_mul_ps(_mm_sqrt_ps(s), sqrt2222f);
            //__m128d sr = _mm_sqrt_pd(s);
            float ar2 = sr.m128_f32[0];
            float ai2 = sr.m128_f32[1];
            if (ai < 0) ai2 = -ai2;
            float br2 = sr.m128_f32[2];
            float bi2 = sr.m128_f32[3];
            if (bi < 0) bi2 = -bi2;

            //cout << sqrt(a) << "    " << r2 <<","<<i2 << endl;
            asqrt = std::complex<float>(ar2, ai2);
            bsqrt = std::complex<float>(br2, bi2);
        }
        __forceinline void  sqrtFast2(const std::complex<double>& a, const std::complex<double>& b, std::complex<double>& asqrt, std::complex<double>& bsqrt)
        {
            const double& ar = a.real();
            const double& ai = a.imag();
            const double& br = b.real();
            const double& bi = b.imag();

            //_mm_csqrt_ps({ (float)ar, (float)ai, (float)br, (float)bi });
            //v0
            //double det = std::sqrt(r*r + i * i);
            //double r2 = sqrt2 * std::sqrt(r + det);
            //double i2 = sqrt2 * std::sqrt(-r + det);
            //v1 
            __m128d det = _mm_sqrt_pd({ ar*ar + ai * ai,br*br + bi * bi });
            double deta = det.m128d_f64[0];
            double detb = det.m128d_f64[1];
            __m128 s{ static_cast<float>(ar + deta), static_cast<float>(-ar + deta),  static_cast<float>(br + detb), static_cast<float>(-br + detb) };
            __m128 sr = _mm_mul_ps(_mm_sqrt_ps(s), sqrt2222);
            //__m128d sr = _mm_sqrt_pd(s);
            double ar2 = sr.m128_f32[0];
            double ai2 = sr.m128_f32[1];
            if (ai < 0) ai2 = -ai2;
            double br2 = sr.m128_f32[2];
            double bi2 = sr.m128_f32[3];
            if (bi < 0) bi2 = -bi2;

            //cout << sqrt(a) << "    " << r2 <<","<<i2 << endl;
            asqrt = std::complex<double>(ar2, ai2);
            bsqrt = std::complex<double>(br2, bi2);
        }
        __forceinline std::complex<double> mul(const std::complex<double>& a, const std::complex<double>& b)
        {
            // this: (a, b), other: (c, d)
            __m128d tmp0, tmp1, tmpR;
            __m128d adata{ a.real(), a.imag() };
            __m128d bdata{ b.real(), b.imag() };

            tmp0 = _mm_shuffle_pd(bdata, bdata, 0x0); // (c, c)
            tmp1 = _mm_shuffle_pd(adata, adata, 0x1); // (b, a)
            tmpR = _mm_mul_pd(adata, tmp0); // (ac, bc)

            tmp0 = _mm_shuffle_pd(bdata, bdata, 0x3); // (d, d)
            tmp1 = _mm_mul_pd(tmp0, tmp1); // (bd, ad)
            tmpR = _mm_addsub_pd(tmpR, tmp1); // (ac - bd, bc + ad)
            return std::complex<double>(tmpR.m128d_f64[0], tmpR.m128d_f64[1]);
        }
    }

}

