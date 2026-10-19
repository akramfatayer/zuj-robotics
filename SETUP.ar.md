<div dir="rtl">

# إعداد البيئة البرمجية لمساق الروبوتات

[English version](SETUP.md)

تستخدم جميع المختبرات والمشروع النهائي **بيئة برمجية واحدة اسمها `zuj-robotics`**،
بالاسم نفسه وبإصدارات الحزم نفسها على حاسوبك المحمول وحاسوبك المنزلي وحواسيب
مختبرات الجامعة؛ فالشيفرة التي تعمل في مكان تعمل في كل مكان.

يمكنك العمل بإحدى ثلاث طرائق، وتستطيع التنقل بينها متى شئت:

| المكان | الإعداد المطلوب | نافذة المحاكاة ثلاثية الأبعاد |
|-------|-----------------|:----------------------------:|
| **Google Colab** | لا شيء؛ افتح رابط Colab الخاص بأي مختبر | لا (رسوم بيانية فقط) |
| **حاسوب مختبر الجامعة** | لا شيء؛ البيئة مثبتة مسبقًا | نعم |
| **حاسوبك المحمول أو الشخصي** | نحو 15 دقيقة، مرة واحدة (هذا الدليل) | نعم |

> **لا تنشئ بيئة خاصة بك على حواسيب المختبر** (مثل `robotics_YourName`)؛
> فالبيئة المشتركة `zuj-robotics` مثبتة مسبقًا على كل حواسيب المختبر.
> انظر قسم [على حواسيب مختبر الجامعة](#على-حواسيب-مختبر-الجامعة).

---

## على حاسوبك المحمول أو الشخصي

يعمل الإعداد على Windows 10/11 وmacOS (معالجات Intel وApple Silicon) وLinux.
تحتاج إلى نحو 3 غيغابايت من المساحة الفارغة واتصال بالإنترنت عند الإعداد الأول.

### الخطوة 1: تثبيت Miniforge (مرة واحدة)

Miniforge برنامج تثبيت صغير ومجاني لأداة `conda` التي تبني البيئة البرمجية.
**هل لديك Anaconda أو Miniconda مثبتًا؟** انتقل مباشرة إلى الخطوة 2 واستخدم
Anaconda Prompt؛ فملف المساق يعتمد على قناة `conda-forge` المجانية فقط.

**نظام Windows**

1. نزّل الملف `Miniforge3-Windows-x86_64.exe` من
   [github.com/conda-forge/miniforge](https://github.com/conda-forge/miniforge#download).
2. شغّله وأبقِ الخيارات الافتراضية ("Just Me" والمجلد الافتراضي).
3. افتح **Miniforge Prompt** من قائمة ابدأ، واستخدم هذه النافذة لتنفيذ كل الأوامر أدناه.

**نظاما macOS وLinux**: افتح الطرفية (Terminal) ونفّذ:

</div>

```bash
curl -L -O "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh"
bash Miniforge3-$(uname)-$(uname -m).sh
```

<div dir="rtl">

أجب بـ `yes` عن سؤال الترخيص وعن سؤال تهيئة conda ("initialize conda")، ثم
**أغلق الطرفية وافتحها من جديد**.

### الخطوة 2: الحصول على شيفرة المساق

باستخدام git (الطريقة المُوصى بها؛ إذ يجلب الأمر `git pull` كل مختبر جديد):

</div>

```bash
git clone https://github.com/akramfatayer/zuj-robotics.git
cd zuj-robotics
```

<div dir="rtl">

دون git: من [صفحة المستودع](https://github.com/akramfatayer/zuj-robotics)
انقر **Code ← Download ZIP**، ثم فكّ ضغط الملف وانتقل بالأمر `cd` إلى المجلد الناتج.

### الخطوة 3: إنشاء البيئة البرمجية (مرة واحدة)

من داخل المجلد `zuj-robotics`:

</div>

```bash
conda env create -f environment.yml
```

<div dir="rtl">

يُنزّل هذا الأمر Python 3.11 وPyBullet وNumPy وMatplotlib وOpenCV،
ويستغرق من 5 إلى 10 دقائق في المرة الأولى.

### الخطوة 4: التحقق من سلامة الإعداد

</div>

```bash
conda activate zuj-robotics
python check_env.py --gui
```

<div dir="rtl">

تُفتح نافذة PyBullet بضع ثوانٍ ويظهر فيها روبوت Husky، ويُكتب `[OK]` في كل سطر.
أصبحت جاهزًا.

### في كل مرة تعمل فيها على مختبر

</div>

```bash
conda activate zuj-robotics
cd zuj-robotics
git pull                              # جلب أحدث ملفات المختبرات
cd labs/lab-01-hello-robot            # المختبر الذي تعمل عليه
python spawn_robot.py
```

<div dir="rtl">

### استخدام VS Code (اختياري)

افتح المجلد `zuj-robotics`، ثم اضغط `Ctrl+Shift+P` (أو `Cmd+Shift+P` على Mac)،
واختر **Python: Select Interpreter**، ثم اختر **zuj-robotics**.

### عند إعلان المدرّس عن تحديث

</div>

```bash
conda activate zuj-robotics
conda env update -f environment.yml --prune
```

<div dir="rtl">

---

## على حواسيب مختبر الجامعة

البيئة البرمجية مثبتة مسبقًا، ولا تحتاج إلى تثبيت أي شيء.

1. انقر نقرًا مزدوجًا على **Robotics Lab** على سطح المكتب؛ فتُفتح طرفية تكون فيها
   البيئة `zuj-robotics` مفعّلة، ويُشغَّل `check_env.py` تلقائيًا.
2. أحضِر ملفات عملك (انظر أدناه)، ثم شغّل المختبر كالمعتاد.

**لا يبقى عملك محفوظًا على حاسوب المختبر.** فقد تجلس إلى حاسوب آخر في المرة القادمة،
كما أن حواسيب المختبر مشتركة مع الشعبة الأخرى. لذا احفظ ملفاتك قبل المغادرة في أحد
الأماكن الآتية:

- **مستودعك الخاص على GitHub** (الأفضل): نفّذ `git push` في نهاية الجلسة و`git pull` في بدايتها.
- **OneDrive أو Google Drive**: ارفع مجلد المختبر في نهاية الجلسة.
- ذاكرة USB، نسخةً احتياطية فقط.

**يُرجى عدم تنفيذ `pip install` أو `conda install` على حواسيب المختبر.** فالبيئة
المشتركة للقراءة فقط كي تعمل بالطريقة نفسها لدى الجميع. وإذا بدا لك أن شيئًا
ما مفقود، فأبلغ المدرّس.

---

## حل المشكلات الشائعة

| المشكلة | الحل |
|---------|------|
| يظهر سطر `pybullet build time: ...` | أمر طبيعي؛ تطبعه PyBullet في كل مرة تبدأ فيها. |
| الأمر `conda` غير معروف | في Windows: استخدم **Miniforge Prompt** (أو Anaconda Prompt) لا موجّه الأوامر العادي. في macOS/Linux: أغلق الطرفية وافتحها من جديد. |
| `ModuleNotFoundError: No module named 'pybullet'` | البيئة غير مفعّلة؛ نفّذ `conda activate zuj-robotics`. |
| يذكر `check_env.py` أن Python تابع للبيئة `base` أو لاسم آخر | الحل نفسه: `conda activate zuj-robotics`. |
| يظهر `EnvironmentNameNotFound` عند التفعيل | لم تكتمل الخطوة 3؛ أعد تنفيذ `conda env create -f environment.yml` من داخل المجلد `zuj-robotics`. |
| يفشل `pip install pybullet` برسالة "Microsoft Visual C++ 14.0 is required" | لا تستخدم pip لتثبيت PyBullet؛ فملف البيئة يثبّت نسخة جاهزة من conda-forge. |
| لا تُفتح نافذة PyBullet | شغّل `python check_env.py` (دون `--gui`). إذا نجح الفحص، فإن برنامج تشغيل الرسوميات يمنع النافذة؛ حدّثه، أو استخدم حواسيب المختبر أو Colab للعرض ثلاثي الأبعاد. |
| حدث عطل وتريد البدء من جديد | نفّذ `conda env remove -n zuj-robotics` ثم أعد الخطوة 3. |
| ما زالت لديك بيئة `robotics_YourName` من الفصل السابق | يمكنك حذفها: `conda env remove -n robotics_YourName`. |

هل ما زالت المشكلة قائمة؟ انشر **المخرجات الكاملة للأمر `python check_env.py`** في
منتدى *Ask the Robot Lab* على Moodle.

</div>
