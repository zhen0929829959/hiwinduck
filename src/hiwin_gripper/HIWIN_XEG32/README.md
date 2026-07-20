# HIWIN_XEG32


進入資料夾
cd /HIWIN_XEG32/src/controller/

編譯程式
cc -o main Hiwin_XEG32_API.c -lmodbus -I ./

生成.so檔
cc -fPIC -shared -o Hiwin_API.so Hiwin_XEG32_API.c -lmodbus -I./



底層程式可見 Hiwin_XEG32_API.c
範例程式可見 Hiwin_API_pretest.py
