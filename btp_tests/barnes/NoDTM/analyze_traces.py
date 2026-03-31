def analyze_trace(filename):
    with open(filename) as f:
        header = f.readline().strip().split()

        core_idx = []
        bank_idx = []

        for i, name in enumerate(header):
            if name.startswith("C_"):
                core_idx.append(i)
            elif name.startswith("B_"):
                bank_idx.append(i)

        core_sum = 0.0
        bank_sum = 0.0
        core_count = 0
        bank_count = 0

        core_max = float("-inf")
        bank_max = float("-inf")

        line_count = 0

        for line in f:
            vals = list(map(float, line.split()))
            line_count += 1

            for i in core_idx:
                v = vals[i]
                core_sum += v
                core_count += 1
                if v > core_max:
                    core_max = v

            for i in bank_idx:
                v = vals[i]
                bank_sum += v
                bank_count += 1
                if v > bank_max:
                    bank_max = v

        core_avg = core_sum / core_count
        bank_avg = bank_sum / bank_count

        return core_avg, core_max, bank_avg, bank_max, line_count


if __name__ == "__main__":

    power_file = "combined_power_total.trace"
    temp_file = "combined_temperature.trace"

    p_core_avg, p_core_max, p_bank_avg, p_bank_max, _ = analyze_trace(power_file)
    t_core_avg, t_core_max, t_bank_avg, t_bank_max, exec_time = analyze_trace(temp_file)

    print("Execution Statistics")
    print(f"Execution time steps, {exec_time}")

    print("\nPower Statistics")
    print(f"Avg core power, {p_core_avg:.4f}")
    print(f"Max core power, {p_core_max:.4f}")
    print(f"Avg bank power, {p_bank_avg:.4f}")
    print(f"Max bank power, {p_bank_max:.4f}")

    print("\nTemperature Statistics")
    print(f"Avg core temperature, {t_core_avg:.4f}")
    print(f"Max core temperature, {t_core_max:.4f}")
    print(f"Avg bank temperature, {t_bank_avg:.4f}")
    print(f"Max bank temperature, {t_bank_max:.4f}")