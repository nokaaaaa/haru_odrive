# これはなに
春ロボ2025用の四輪オムニの足回り(作成中) 
いまのところtwistをsubscribeして足回りを動かす
# 使い方
oodrivetoolでキャリブレーション 
下のようにconfig.jsonを書く 
右前、左前、左後ろ、右前の順に0,1,2,3になるようにする
```
{
    "motors": [
      {
        "serial_number": "346031583432",
        "axis": 1,
        "topic_name": "/motor0_speed"
      },
      {
        "serial_number": "2071307C4252",
        "axis": 1,
        "topic_name": "/motor1_speed"
      },
      {
        "serial_number": "2071307C4252",
        "axis": 0,
        "topic_name": "/motor2_speed"
      },
      {
        "serial_number": "346031583432",
        "axis": 0,
        "topic_name": "/motor3_speed"
      }
    ]
  }
```
下を実行する
```
ros2 run od_ros od --ros-args --param config_file:=[config.jsonのpath]
```
https://github.com/nokaaaaa/haru_gui  
これを使うといい感じにスマホをコントローラできる
