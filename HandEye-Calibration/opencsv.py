import csv

presets = []
with open('z1_controller/config/savedArmStates.csv', newline='') as f:
    reader = csv.reader(f)
    count = 0
    for row in reader:
        if not row or row[0].startswith('#'):
            continue
        name = row[0].strip()
        # 跳过 forward（如果仍需要）
        if name == 'forward':
            continue

        values = [float(x) for x in row[1:1+6]]
        presets.append((name, values))

        count += 1
        if count >= 20:
            break

print(presets)
