
import tkinter as tk
from tkinter import ttk
from subprocess import Popen, PIPE
import threading
import os
import sys

floor=0
floor_flag=True

thread_mapping = threading.Thread()
def startMapping():
    # 切换布局
    # 启动建图
    global floor
    global thread_mapping
    mapping_fuction = combobox3.get()
    if mapping_fuction == "fast":
        label0.config(text="正在建图...")
        label1.grid_remove()
        entry1.grid_remove()
        label2.grid_remove()
        combobox2.grid_remove()
        label3.grid_remove()
        combobox3.grid_remove()
        label4.grid_remove()
        combobox4.grid_remove()
        button1.grid_remove()

        label_f_1.grid(row=1, column=0, columnspan=3)
        label_f_2.grid(row=2, column=0, columnspan=3)
        button_f_1.grid(row=3, column=0, columnspan=3)
        button_f_2.grid(row=4, column=0, columnspan=3)
        text.grid(row=5, column=0, columnspan=3)
    elif mapping_fuction == "indoor" or mapping_fuction == "outdoor":
        label0.config(text="正在建图...")
        label1.grid_remove()
        entry1.grid_remove()
        label2.grid_remove()
        combobox2.grid_remove()
        label3.grid_remove()
        combobox3.grid_remove()
        label4.grid_remove()
        combobox4.grid_remove()
        button1.grid_remove()

        label_io_1.grid(row=1, column=0, columnspan=3)
        label_io_2.grid(row=2, column=0, columnspan=3)
        button_io_3.grid(row=3, column=0, columnspan=1)
        default_value_io_1 = tk.StringVar(value=str(floor+1))
        entry_io_1.config(textvariable=default_value_io_1)
        entry_io_1.grid(row=3, column=1, columnspan=2)
        button_io_4.grid(row=4, column=0, columnspan=3)
        button_f_2.grid(row=5, column=0, columnspan=3)
        button_io_3.config(text="指定新楼层：")
        text.grid(row=6, column=0, columnspan=3)
    def mappingFunction():
        # print("Start mapping.sh")
        process1 = Popen(["bash", os.path.dirname(os.path.abspath(__file__)) + "/mapping.sh", entry1.get(), combobox2.get(), combobox3.get(), ("true" if (combobox4.get() == "y") else "false")], stdout=PIPE, stderr=PIPE, text=True)
        # 逐行读取输出
        for line in process1.stdout:
            text.insert(tk.END, line)
            text.see(tk.END)  # 自动滚动到最新输出
        process1.wait()
        # while(True):
        #     output, err = process1.communicate()
        #     for line in output:
        #         text.insert(tk.END, line)
        #         text.see(tk.END)  # 自动滚动到最新输出
        #     if not process1.wait():
        #         break
        # print("Finish mapping.sh")
    thread_mapping = threading.Thread(target=mappingFunction)
    thread_mapping.start()
        


    

def newFloorFast():
    global floor
    floor = floor + 1
    label_f_1.config(text="当前楼层："+str(floor))
    grid_map_file_name="jueying"
    if floor != 0:
        grid_map_file_name="jueying" + str(floor)
    label_f_2.config(text="当前楼层栅格地图名："+grid_map_file_name)

    def newFloorFastFunction():
        process2 = Popen(["bash", os.path.dirname(os.path.abspath(__file__)) + "/occupancy_mapping.sh"], stdout=PIPE, stderr=PIPE, text=True)
        # 逐行读取输出
        for line in process2.stdout:
            text.insert(tk.END, line)
            text.see(tk.END)  # 自动滚动到最新输出
        process2.wait()
    threading.Thread(target=newFloorFastFunction).start()

def newFloor():
    global floor
    global floor_flag
    floor_flag=False
    # floor = floor + 1
    floor = int(entry_io_1.get())
    label_io_1.config(text="当前楼层："+str(floor))
    grid_map_file_name="jueying"
    if floor != 0:
        grid_map_file_name="jueying" + str(floor)
    label_io_2.config(text="当前楼层栅格地图名："+grid_map_file_name)
    button_io_3.config(text="指定新楼层：")
    default_value_io_1 = tk.StringVar(value=str(floor+1))
    entry_io_1.config(textvariable=default_value_io_1)

    # def newFloorFunction():
    process3 = Popen(["bash", os.path.dirname(os.path.abspath(__file__)) + "/set_floor_label.sh", str(floor)], stdout=PIPE, stderr=PIPE, text=True)
    # 逐行读取输出
    for line in process3.stdout:
        text.insert(tk.END, line)
        text.see(tk.END)  # 自动滚动到最新输出
    process3.wait()
    # threading.Thread(target=newFloorFunction).start()

def stopMapping():
    def stopMappingFunction():
        finish_flag = False
        global thread_mapping
        # def stopMappingFunction():
        process = Popen(["bash", os.path.dirname(os.path.abspath(__file__)) + "/mapping_stop.sh"], stdout=PIPE, stderr=PIPE, text=True)
        # 逐行读取输出
        for line in process.stdout:
            text.insert(tk.END, line)
            text.see(tk.END)  # 自动滚动到最新输出
        process.wait()
        # finish_flag=True
        # thread_mapping_stop = threading.Thread(target=stopMappingFunction)
        # thread_mapping_stop.start()
        # thread_mapping_stop.join()
        thread_mapping.join()

        # process4 = Popen(["bash", os.path.dirname(os.path.abspath(__file__)) + "/mapping_stop.sh"], stdout=PIPE, stderr=PIPE, text=True)
        # # 逐行读取输出
        # for line in process4.stdout:
        #     text.insert(tk.END, line)
        #     text.see(tk.END)  # 自动滚动到最新输出
        # process4.wait()

        label0.config(text="结束建图程序...")
        # os.sleep(5)

        def constructFullMapFunction():
            process = Popen(["bash", os.path.dirname(os.path.abspath(__file__)) + "/construct_full_map.sh"], stdout=PIPE, stderr=PIPE, text=True)
            # 逐行读取输出
            for line in process.stdout:
                text.insert(tk.END, line)
                text.see(tk.END)  # 自动滚动到最新输出
            process.wait()
        def constructGrid2DMapFunction():
            process = Popen(["bash", os.path.dirname(os.path.abspath(__file__)) + "/occupancy_mapping.sh", "offline"], stdout=PIPE, stderr=PIPE, text=True)
            # 逐行读取输出
            for line in process.stdout:
                text.insert(tk.END, line)
                text.see(tk.END)  # 自动滚动到最新输出
            process.wait()

        mapping_fuction = combobox3.get()
        if mapping_fuction == "fast":
            label0.config(text="建图完成")
            label_f_1.grid_remove()
            label_f_2.grid_remove()
            button_f_1.grid_remove()
            button_f_2.grid_remove()
            text.grid_remove()
            button_io_5.grid(row=1, column=0, columnspan=3)
        elif mapping_fuction == "indoor" or mapping_fuction == "outdoor":
            label0.config(text="正在生成完整点云地图...")
            label_io_1.grid_remove()
            label_io_2.grid_remove()
            button_io_3.grid_remove()
            entry_io_1.grid_remove()
            button_io_4.grid_remove()
            button_f_2.grid_remove()
            button_io_3.grid_remove()
            text.grid(row=1, column=0, columnspan=3)
            text.see(tk.END)
            # thread_construct_full_map = threading.Thread(target=constructFullMapFunction)
            # thread_construct_full_map.start()
            # thread_construct_full_map.join()
            # label0.config(text="正在生成完整栅格地图...")
            # thread_construct_grid_map = threading.Thread(target=constructGrid2DMapFunction)
            # thread_construct_grid_map.start()
            # thread_construct_grid_map.join()

            process5 = Popen(["bash", os.path.dirname(os.path.abspath(__file__)) + "/construct_full_map.sh"], stdout=PIPE, stderr=PIPE, text=True)
            # 逐行读取输出
            for line in process5.stdout:
                text.insert(tk.END, line)
                text.see(tk.END)  # 自动滚动到最新输出
            process5.wait()

            label0.config(text="正在生成栅格地图...")
            process6 = Popen(["bash", os.path.dirname(os.path.abspath(__file__)) + "/occupancy_mapping.sh", "offline"], stdout=PIPE, stderr=PIPE, text=True)
            # 逐行读取输出
            for line in process6.stdout:
                text.insert(tk.END, line)
                text.see(tk.END)  # 自动滚动到最新输出
            process6.wait()
            label0.config(text="建图完成")
            button_io_5.grid(row=1, column=0, columnspan=3)
            text.grid(row=2, column=0, columnspan=3)
            # text.see(tk.END)
            # text.grid_remove()
    threading.Thread(target=stopMappingFunction).start()

    




def notSpecifyFloor():
    global floor_flag
    floor_flag=False
    label_io_1.config(text="当前楼层：不指定")
    label_io_2.config(text="当前楼层栅格地图名：不生成栅格地图")

    # def newFloorFunction():
    process = Popen(["bash", os.path.dirname(os.path.abspath(__file__)) + "/set_floor_label.sh", "-1"], stdout=PIPE, stderr=PIPE, text=True)
    # 逐行读取输出
    for line in process.stdout:
        text.insert(tk.END, line)
        text.see(tk.END)  # 自动滚动到最新输出
    process.wait()
    # threading.Thread(target=newFloorFunction).start()


def closeWindow():
    sys.exit()


# 创建窗口
root = tk.Tk()
root.title("建图")

# 准备建图阶段，需指定地图名称、是否立即生效、建图方法、是否打开rviz等
label0 = tk.Label(root, text="准备建图，请填写必要内容")
label0.grid(row=0, column=0, columnspan=3)


label1 = tk.Label(root, text="地图名称：")
label1.grid(row=1, column=0)

default_value1 = tk.StringVar(value="ProjectName-LocationName")
entry1 = tk.Entry(root, textvariable=default_value1)
entry1.grid(row=1, column=1, columnspan=2)


label2 = tk.Label(root, text="立即生效？")
label2.grid(row=2, column=0)

combobox2 = ttk.Combobox(root)
# 设置下拉菜单的选项
combobox2['values'] = ('y', 'n')
# 设置默认显示的值，values的索引，此处为'选项2'
combobox2.current(0)
# 将下拉菜单放置到主窗口
combobox2.grid(row=2, column=1, columnspan=2)

label3 = tk.Label(root, text="建图方法：")
label3.grid(row=3, column=0)

combobox3 = ttk.Combobox(root)
# 设置下拉菜单的选项
combobox3['values'] = ('fast', 'indoor', 'outdoor')
# 设置默认显示的值，values的索引，此处为'选项2'
combobox3.current(0)
# 将下拉菜单放置到主窗口
combobox3.grid(row=3, column=1, columnspan=2)

label4 = tk.Label(root, text="打开rviz？")
label4.grid(row=4, column=0)

combobox4 = ttk.Combobox(root)
# 设置下拉菜单的选项
combobox4['values'] = ('y', 'n')
# 设置默认显示的值，values的索引，此处为'选项2'
combobox4.current(0)
# 将下拉菜单放置到主窗口
combobox4.grid(row=4, column=1, columnspan=2)

button1 = tk.Button(root, text="开始建图", command=startMapping)
button1.grid(row=5, column=0, columnspan=3)


# 针对fast模式建图
label_f_1 = tk.Label(root, text="当前楼层："+str(floor))
#label_f_1.grid(row=1, column=0)
grid_map_file_name="jueying"
if floor != 0:
    grid_map_file_name="jueying" + str(floor)
label_f_2 = tk.Label(root, text="当前楼层栅格地图名："+grid_map_file_name)
#label_f_2.grid(row=1, column=1, columnspan=2)

button_f_1 = tk.Button(root, text="新楼层", command=newFloorFast)
#button_f_1.grid(row=2, column=0, columnspan=3)

button_f_2 = tk.Button(root, text="结束建图", command=stopMapping)
#button_f_2.grid(row=3, column=0, columnspan=3)

# 针对indoor/outdoor模式建图
label_io_1 = tk.Label(root, text="当前楼层："+str(floor))
label_io_2 = tk.Label(root, text="当前楼层栅格地图名："+grid_map_file_name)

button_io_3 = tk.Button(root, text="指定新楼层：", command=newFloor)
default_value_io_1 = tk.StringVar(value=str(floor+1))
entry_io_1 = tk.Entry(root, textvariable=default_value_io_1)
button_io_4 = tk.Button(root, text="不指定楼层", command=notSpecifyFloor)

button_io_5 = tk.Button(root, text="关闭", command=closeWindow)





text = tk.Text(root)
# text.pack()


root.mainloop()