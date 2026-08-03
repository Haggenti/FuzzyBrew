import argparse
import csv
import sys

import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


def exponential_func(x, a, b, c):
    """Exponential function y = a * e^(bx) + c"""
    return a * np.exp(b * x) + c


def linear_func(x, a, b):
    """Linear function y = a*x + b"""
    return a * x + b


def quadratic_func(x, a, b, c):
    """Quadratic function y = a*x^2 + b*x + c"""
    return a * x ** 2 + b * x + c


def power_func(x, a, b, c):
    """Power function y = a * x^b + c"""
    return a * np.power(x, b) + c


def r_squared(y, y_pred):
    ss_res = np.sum((y - y_pred) ** 2)
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0


def rmse(y, y_pred):
    return np.sqrt(np.mean((y - y_pred) ** 2))


def try_fit(func, x, y, p0, bounds, name):
    try:
        popt, _ = curve_fit(func, x, y, p0=p0, bounds=bounds, maxfev=10000)
        y_pred = func(x, *popt)
        return {
            "name": name,
            "params": popt,
            "y_pred": y_pred,
            "rmse": rmse(y, y_pred),
            "r2": r_squared(y, y_pred),
            "func": func,
        }
    except Exception:
        return None


def get_positive_int(prompt):
    while True:
        raw = input(prompt).strip()
        try:
            value = int(raw)
            if value <= 0:
                raise ValueError
            return value
        except ValueError:
            print("Please enter a positive integer.")


def get_validated_float(prompt, min_value=None, max_value=None):
    while True:
        raw = input(prompt).strip()
        try:
            value = float(raw)
        except ValueError:
            print("Please enter a valid number.")
            continue

        if min_value is not None and value < min_value:
            print(f"Value must be >= {min_value}.")
            continue
        if max_value is not None and value > max_value:
            print(f"Value must be <= {max_value}.")
            continue

        return value


def get_data_points():
    """Gets data points from the user"""
    n_points = get_positive_int("How many measurement points? ")
    if n_points < 3:
        print("At least 3 measurement points are required for the fit.")
        return get_data_points()

    temperatures = []
    pwm_values = []

    for i in range(n_points):
        print(f"\nPoint {i+1}:")
        temp = get_validated_float("Temperature (°C): ", min_value=-273.15)
        pwm = get_validated_float("PWM maintain (%): ", min_value=0.0, max_value=100.0)
        temperatures.append(temp)
        pwm_values.append(pwm)

    return np.array(temperatures), np.array(pwm_values)


def get_data_points_from_csv(file_path):
    """Reads data points from a CSV file"""
    temperatures = []
    pwm_values = []

    try:
        with open(file_path, newline='') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                if not row or all(not cell.strip() for cell in row):
                    continue

                try:
                    temp = float(row[0])
                    pwm = float(row[1])
                except (ValueError, IndexError):
                    continue

                if temp < -273.15:
                    raise ValueError(f"Invalid temperature value: {temp}")
                if pwm < 0.0 or pwm > 100.0:
                    raise ValueError(f"Invalid PWM value: {pwm}")

                temperatures.append(temp)
                pwm_values.append(pwm)
    except FileNotFoundError:
        raise FileNotFoundError(f"CSV file not found: {file_path}")

    if len(temperatures) < 3:
        raise ValueError("CSV file must contain at least 3 valid measurement points.")

    return np.array(temperatures), np.array(pwm_values)


def main():
    parser = argparse.ArgumentParser(
        description="Coefficient calculator for the maintenance curve"
    )
    parser.add_argument(
        "--file",
        help="Path to a CSV file with temperature and PWM columns",
        default=None,
    )
    args = parser.parse_args()

    print("Coefficient calculator for the maintenance curve")
    print("Candidate models: linear, quadratic, power, exponential")
    print("---------------------------------------------------")

    if args.file:
        try:
            temperatures, pwm_values = get_data_points_from_csv(args.file)
        except Exception as exc:
            print(f"Error reading CSV: {exc}")
            sys.exit(1)
    else:
        temperatures, pwm_values = get_data_points()

    models = [
        {
            "name": "linear",
            "func": linear_func,
            "p0": [0.0, np.mean(pwm_values)],
            "bounds": ([-np.inf, -np.inf], [np.inf, np.inf]),
        },
        {
            "name": "quadratic",
            "func": quadratic_func,
            "p0": [0.0, 0.0, np.mean(pwm_values)],
            "bounds": ([-np.inf, -np.inf, -np.inf], [np.inf, np.inf, np.inf]),
        },
        {
            "name": "exponential",
            "func": exponential_func,
            "p0": [1.0, -0.01, np.min(pwm_values)],
            "bounds": ([-np.inf, -np.inf, -np.inf], [np.inf, np.inf, np.inf]),
        },
    ]

    if np.min(temperatures) > 0:
        models.append(
            {
                "name": "power",
                "func": power_func,
                "p0": [1.0, 1.0, np.min(pwm_values)],
                "bounds": ([-np.inf, -np.inf, -np.inf], [np.inf, np.inf, np.inf]),
            }
        )

    fitted_models = []
    for model in models:
        fit = try_fit(
            model["func"],
            temperatures,
            pwm_values,
            p0=model["p0"],
            bounds=model["bounds"],
            name=model["name"],
        )
        if fit is not None:
            fitted_models.append(fit)

    if not fitted_models:
        print("\nError: Unable to fit any model to the provided data.")
        sys.exit(1)

    fitted_models.sort(key=lambda item: item["rmse"])
    best_model = fitted_models[0]

    print("\nFit quality for each model:")
    for model in fitted_models:
        print(f"- {model['name']}: RMSE={model['rmse']:.4f}, R²={model['r2']:.4f}")

    print("\nSelected best model:")
    print(f"Model: {best_model['name']}")
    print(f"RMSE: {best_model['rmse']:.4f}")
    print(f"R²: {best_model['r2']:.4f}")
    print("Parameters:")
    print(", ".join(f"{value:.5f}" for value in best_model["params"]))

    temp_curve = np.linspace(min(temperatures), max(temperatures), 200)
    pwm_curve = best_model["func"](temp_curve, *best_model["params"])

    plt.figure(figsize=(10, 6))
    plt.scatter(temperatures, pwm_values, color="blue", label="Measured points")
    plt.plot(temp_curve, pwm_curve, "r-", label=f"Best fit: {best_model[name]}")
    plt.xlabel("Temperature (°C)")
    plt.ylabel("PWM maintain (%)")
    plt.title("Temperature maintenance curve")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
