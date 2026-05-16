# 板载 LED 闪烁测试例程

## 编译和运行

1. 激活idf
2. 切换到esp32p4
  ```shell
  idf.py set-target esp32p4
  ```
3. 编译代码
  ```shell
  idf.py build
  ```
4. 烧录固件
  ```shell
  idf.py flash
  ```

## 运行结果

运行后，板载的RGB灯闪烁