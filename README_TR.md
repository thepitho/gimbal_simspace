# AKGÖZ GIMBAL — 2 eksen LOS stabilizasyon modeli (Simscape Multibody)

Hedef: MATLAB R2025b + Simulink + Simscape + Simscape Multibody.
Kontrol yapısı, sensör modeli ve parametre kaynakları gimbal-for-fun projesinden
(`sim/gimbalsim/control.py`, `sensors.py`, `params/gimbal.yaml`) uyarlanmıştır.

## Hızlı başlangıç (MATLAB, bu klasörde)

    out = run_demo;            % stabilized modu: modeli kurar, 12 s simüle eder, çizer
    T   = akgoz_compare;       % PI / PI+lead / PI+DOB / PI+lead+DOB RMS LOS tablosu
    check_model                % üç modu 0.5 s derleyip çalıştırır

Diğer modlar: `run_demo('torque')` (göreli PD, stabilize değil),
`run_demo('kinematic')` (yalnızca animasyon).

## Mekanik ve kütle özellikleri
Roll ekseni yok: azimuth (+Z) ve elevation (−Y; + = burun yukarı). İki eksen O
noktasında kesişir.

`make_akgoz_urdf.py` iki geometri üretebilir (`AKGOZ_GEOMETRY` ortam değişkeni):
- **cad** (varsayılan): gimbal-for-fun `cad/turret.py` CAD'i. Görünüş
  (`meshes/cad/*.stl`, renkli), gövde kütleleri, COM'lar, tam atalet tensörleri ve
  EO/IR pencere konumları `cad_ref/massprops.json`'dan. Top Ø189 mm, O montaj
  düzleminin 186 mm altında, toplam 2.805 kg (baş 1.513, yoke 0.642, taban 0.649 kg).
  İzolatörler rijit kabul edilir. Baş montajına 0.5 mm / 0.3 mm kalıntı dengesizlik
  eklenir (placeholder). Eksen ataletleri: J_el = 0.0039, J_az(0) = 0.0099 kg·m².
- **akgoz**: Akgöz fotoğraflarından oranlanmış görünüş (huni omuz, kulakçıklar,
  küresel baş, üç pencere). Ölçü ve kütleler tahmindir.

Her iki mod da kontrolcünün kullandığı `akgoz_inertia.m` dosyasını yazar.
CAD değişirse: `<gimbal-for-fun>/.venv/bin/python -I tools/export_ref_cad.py
<gimbal-for-fun>/cad/out` ve ardından `python3 make_akgoz_urdf.py`; sonra model
yeniden kurulur. Akgöz'ün kendi CAD'i gelince aynı yol kullanılabilir.

Dünya: X ileri, Y sol, Z yukarı (NED değil). Kamera optik ekseni +X.
Taban dönüşü: R = Rz(yaw)·Ry(pitch)·Rz(az)·Ry(−el).

## Stabilized modu — blok şeması

    base_motion ─► taban yaw/pitch (manevra + 2–40 Hz titreşim + 85 Hz ton)
                                   │
    Simscape plant (sürekli) ──────┴──► q, w ─► truth_LOS ─► gerçek baş hızı ─► gyro_STIM210 ─┐
          ▲                                       └─► los_err (gerçek LOS hatası, log)          │
          │                                                                         1 kHz       ▼
    actuator_friction ◄─ akım döngüsü 500 Hz ◄─ i_cmd ◄─ LOS_Controller_1kHz ◄─ gyro, LOS_estimator (INS×encoder, LOS_cmd)

`LOS_Controller_1kHz`, tamamen standart Simulink bloklarıyla kurulur
(`akgoz_build_controller.m`; Gain, Sum, Product, Switch, Saturation, Discrete-Time
Integrator, Discrete Transfer Fcn, Difference, Unit Delay). Her eksen bir satır:

    LOS komutu ─► [dış döngü: LOS konum, P 3 Hz, slew ≤ 60°/s] ─► LOS hız komutu
              ─► [iç (slave) döngü: gyro hız PI] ─► [lead] ─► ×J ekseni (az: ×sec el)
              ─► − DOB tahmini ─► tork LPF 150 Hz ─► i = τ/Kt, |i| ≤ 2.65 A

- İç döngü PI: α = ωc·e + (ωc²/4)∫e, τ = J·α (atalet ölçekli). Anti-windup: integratör
  durumu tork limitine karşılık gelen değerde kırpılır. Az kanalında J_az(el)·sec(el),
  sec en fazla 5. Slew sınırı eksen başına.
- Lead: C(s) = k(s/ωz+1)/(s/ωp+1), 30° faz ωc'de, |C(jωc)| = 1, Tustin 1 kHz.
- Kesim frekansı: lead kapalıyken 25 Hz (faz payı ≈ 43°), açıkken 35 Hz (≈ 61°).
  Bkz. `tools/axis_margin_check.py`.
- DOB: d̂ = Q(z)[J·Δω/Δt − τ_uygulanan], τ ← τ − d̂, Q birinci dereceden 20 Hz.
  40 Hz, lead + 35–40 Hz kesimle kararsızlaşıyor (`tools/axis_sim_check.py`).
- Dış döngü girdisi `LOS_estimator` (Sensors içinde, MATLAB Function: yalnızca 3B
  geometri): kamera çerçevesinde dönme vektörü LOS hatası (ESKF yok; taban açısı =
  ideal INS, eklem açısı = 16-bit encoder).
- `DOB_on`, `LEAD_on` Constant blokları ile aç/kapat.

Sensör ve aktüatör (1 kHz ayrık, plant sürekli):
- Gyro STIM210 (`akgoz_gyro.m`): 262 Hz bant, 2.7 ms gecikme, ARW 0.15°/√h,
  Gauss–Markov bias 0.3°/h, açılış biası, ölçek hatası, eksen kaçıklığı, 0.22°/h kuantalama.
- Motor Celera Omni+ OPH-060-013-D: Kt 0.168 N·m/A, 2.65 A; FOC akım döngüsü 500 Hz (1. derece).
- `akgoz_actuator.m`: Stribeck + viskoz sürtünme (tanh yumuşatma), cogging, el kablo yayı.

Senaryo: 2 s'de LOS el −20°, 6 s'de az +15°, RMS penceresi t ≥ 8 s.

Tek eksenli Python ön kontrolü (Simscape değil, yalnızca eğilim; GEOMETRY=cad ataletiyle):

| Yapılandırma | RMS [µrad] | tepe [µrad] |
|---|---:|---:|
| PI (25 Hz) | 47.8 | 274 |
| PI + lead (35 Hz) | 43.5 | 243 |
| PI + DOB (20 Hz) | 25.1 | 175 |
| PI + lead + DOB | 23.6 | 143 |

Hafif başta (J_el 0.0039) aynı sürtünme torku daha çok ivme yaratır; hatanın ana
kaynağı Stribeck sürtünmesidir (placeholder değerler).

Çıkışlar: `out.los_err` [az; el] rad (gerçek), `out.gyro_meas`, `out.gyro_true`,
`out.ctrl_diag` = [w_cmd(2); w_meas(2); d̂(2); err_tahmin(2); τ_DOB_öncesi(2)],
`out.*_i_cmd` [A], `out.*_torque` [N·m] (net, sürtünme sonrası), `out.*_q`, `out.*_w`.

## Parametreler ve dürüstlük
Tüm sayılar `akgoz_params.m` içinde, her biri kaynak etiketli
(`datasheet:` / `calc:` / `placeholder`). Motor ve gyro değerleri datasheet'ten;
sürtünme, titreşim spektrumu, kazançlar, akım döngüsü bant genişliği placeholder.
CAD kütleleri gimbal-for-fun tasarımına aittir (Akgöz'ün kendisine değil). Modelde yok: izolatörler, INS hatası / ESKF,
LuGre (gerçek stiction), mekanik limitler, yapısal esneklik, motor elektrik dinamiğinin
detayı, hesaplama gecikmesi. Çıkan µrad değerleri tasarım sonucu değildir.

Kontrolcü blokları ve sensör/aktüatör blok kodları modele **kurulum anında** oluşturulur
(`akgoz_build_controller.m`, `akgoz_*.m`). Kazançlar blokların içine sayı olarak yazılır.
Bu yüzden `akgoz_params.m`'deki kontrol kazançlarını veya bir `akgoz_*.m` dosyasını
değiştirdikten sonra modeli yeniden kurun ya da bloğu doğrudan Simulink'te düzenleyin.
Gyro, sürtünme, titreşim gibi MATLAB Function bloklarının parametreleri ise
`akgoz_params.m`'den çalışma anında okunur.

## Yeniden kurma
Builder var olan .slx'in üzerine yazmaz. Önce dosyayı taşıyın ve bellekten kapatın:

    close_system('akgoz_gimbal_stabilized',0); delete akgoz_gimbal_stabilized.slx
    out = run_demo;

## Doğrulama durumu
Bu paketin hazırlandığı ortamda MATLAB yok; model burada derlenmedi/çalıştırılmadı.
Kontrol edilenler: URDF ağacı ve ataletleri, kontrol yasasının tek eksenli Python
simülasyonu ve ayrık zaman faz payı hesabı (`tools/`). Hata olursa tam metni paylaşın.
Sürüme bağlı maske adları için: `get_param('akgoz_gimbal_stabilized/azimuth','DialogParameters')`.

## Kaynaklar
https://www.mathworks.com/help/sm/ref/smimport.html
https://www.mathworks.com/help/sm/ug/urdf-import.html
https://www.mathworks.com/help/simscape/ref/simulinkpsconverter.html
https://www.mathworks.com/help/simulink/slref/matlabfunctionconfiguration.html
