# 导入所需库
import pandas as pd
import matplotlib.pyplot as plt

# -------------------------- 需要你填写/修改的部分 START --------------------------
# 1. CSV文件路径（必填）：替换为你的results.csv实际路径
CSV_FILE_PATH = ""  # 例如："D:/project/results.csv"

# 2. 图表显示配置（可选）：可根据需要修改标题、坐标轴标签、保存路径等
CHART_SAVE_PATH = ""  # 图表保存路径（含文件名）
CHART_DPI = 300  # 图表分辨率，数值越高越清晰
FONT_SIZE = 12  # 图表字体大小

# 3. CSV编码配置（可选）：如果读取文件出现乱码，可尝试修改为 'gbk' 或 'gb2312'
CSV_ENCODING = 'utf-8'


# -------------------------- 需要你填写/修改的部分 END ----------------------------

def plot_loss_curves():
    # 设置中文字体（如果需要显示中文，取消下面两行注释并根据系统调整字体）
    # plt.rcParams['font.sans-serif'] = ['SimHei']  # Windows系统
    # plt.rcParams['axes.unicode_minus'] = False    # 解决负号显示问题

    # 读取CSV文件（核心修改点：从read_excel改为read_csv）
    try:
        df = pd.read_csv(CSV_FILE_PATH, encoding=CSV_ENCODING)
    except FileNotFoundError:
        print(f"错误：未找到文件，请检查路径是否正确：{CSV_FILE_PATH}")
        return
    except UnicodeDecodeError:
        print(f"错误：文件编码不匹配，尝试修改CSV_ENCODING为'gbk'或'gb2312'")
        return
    except Exception as e:
        print(f"读取文件时出错：{e}")
        return

    # 提取x轴数据（epoch，表格第一列）
    epoch = df.iloc[:, 0]  # iloc[:,0] 表示取所有行的第一列

    # 定义需要绘制的loss类型（对应表格列名）
    loss_types = [
        ("box_loss", "train/box_loss", "val/box_loss"),
        ("cls_loss", "train/cls_loss", "val/cls_loss"),
        ("dfl_loss", "train/dfl_loss", "val/dfl_loss")
    ]

    # 创建3个子图（1行3列），用于分别显示三种loss
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle("Train vs Val Loss Curves", fontsize=FONT_SIZE + 4)

    # 遍历每个loss类型，绘制对应的train和val曲线
    for idx, (loss_name, train_col, val_col) in enumerate(loss_types):
        ax = axes[idx]

        # 检查列名是否存在
        if train_col not in df.columns or val_col not in df.columns:
            ax.text(0.5, 0.5, f"列名不存在：\n{train_col} 或 {val_col}",
                    ha='center', va='center', transform=ax.transAxes)
            ax.set_title(f"{loss_name} (数据缺失)")
            continue

        # 提取train和val的loss数据
        train_loss = df[train_col]
        val_loss = df[val_col]

        # 绘制曲线
        ax.plot(epoch, train_loss, label=f"Train {loss_name}", color="blue", linewidth=2)
        ax.plot(epoch, val_loss, label=f"Val {loss_name}", color="orange", linewidth=2, linestyle="--")

        # 设置子图标题和坐标轴标签
        ax.set_title(f"{loss_name} Curve", fontsize=FONT_SIZE + 1)
        ax.set_xlabel("Epoch", fontsize=FONT_SIZE)
        ax.set_ylabel("Loss Value", fontsize=FONT_SIZE)
        ax.legend(fontsize=FONT_SIZE - 1)
        ax.grid(True, alpha=0.3)  # 添加网格线，增强可读性

    # 调整子图间距，避免重叠
    plt.tight_layout()

    # 保存图表
    plt.savefig(CHART_SAVE_PATH, dpi=CHART_DPI, bbox_inches="tight")
    print(f"图表已保存至：{CHART_SAVE_PATH}")

    # 显示图表（运行程序时会弹出窗口）
    plt.show()


if __name__ == "__main__":
    plot_loss_curves()