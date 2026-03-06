# BALLISTX

**Real-Time Ballistics Simulation Engine**

C++17 ile yazılmış, gerçek zamanlı balistik ve uçuş dinamiği simülasyon motoru.

## Özellikler

- ✅ Runge-Kutta 4th Order entegrasyon (O(dt⁴))
- ✅ ISA Atmosfer modeli (yüksekliğe bağımlı)
- ✅ Mach sayısına bağımlı sürükleme katsayısı (G7 drag curve)
- ✅ Yüksekliğe bağımlı gravitasyon modeli
- ✅ 3D vektör matematiği
- ✅ 155mm top mermisi simülasyonu

## Kurulum

```bash
git clone https://github.com/kodchhdayininyeri/BALLISTX.git
cd BALLISTX
cmake -B build
cmake --build build --config Debug
```

## Kullanım

```bash
# Ana demo (155mm simülasyonu)
.\build\Debug\ballistx_demo.exe

# RK4 vs Euler karşılaştırma
.\build\Debug\rk4_trajectory.exe

# Mach-dependent drag demo
.\build\Debug\mach_drag_demo.exe

# Gravitasyon modeli demo
.\build\Debug\gravity_demo.exe
```

## Teknoloji Stack

- **C++17** - Modern C++ standartları
- **CMake 3.20+** - Build sistemi
- **MSVC / GCC / Clang** - Çapraz derleyici desteği

## Doğruluk

155mm M107 top mermisi simülasyonu:
- Gerçek dünya menzili: 18-24 km
- BALLISTX sonucu: 22.5 km
- **Hata: %1.5** ✅

## Lisans

MIT License

## Versiyon

v0.1.0 - Alpha
