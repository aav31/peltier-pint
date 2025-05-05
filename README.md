# 🍺 peltier-pint

An unnecessarily complicated and wildly inefficient way to chill a pint.

Because when you’re holding a perfectly drinkable lager at 10°C, the only reasonable next step is to boot up a full ROS2 stack, engage two relays, and fire 250 watts at it until it submits to 6°C perfection.


![demo.gif](demo.gif)  
*Above: A surprisingly effective thermodynamic crime in progress.*

## 🧪 Why I Made This

This was my way of doing something *fun and functional* with ROS2, beyond turtlesim and generic tutorials. I wanted to combine hardware control, topic communication, and some real-world feedback.

This is not a production-grade beverage cooler. But it is a project that brought together sensors, control loops, and reckless thermoelectric ambition in one satisfying weekend build.

Yes, I could’ve just bought a mini fridge or used an ice bucket. But where’s the fun in that?

## 🧊 The Cooling Strategy

- **Target Temperature**: 5–7°C
- **Control Method**: Classic *bang-bang* control — if it's too warm, the system turns on the cooling hardware; if it's cold enough, it turns it off.
- **Hardware**:
  - 4x Peltier modules
  - 4x Cooling fans
  - 2x Relays switching a 12V / 250W PSU
  - 1x DS18B20 sensor for temperature feedback
  - 1x Raspberry Pi 4
- **Software**: 2 ROS2 nodes — one publishes the temperature, the other controls the cooling.

## 📂 Files & Media

- **[temperature_publisher.py](src/peltier_pint/peltier_pint/temperature_publisher.py)** – Reads from the DS18B20 sensor and publishes data to the `/temperature` topic.
- **[relay_control_subscriber.py](src/peltier_pint/peltier_pint/relay_control_subscriber.py)** – Subscribes to `/temperature`, then turns the relays on/off depending on whether the reading is within range.
- **[start_cooler.sh](start_cooler.sh)** – Startup script which: enables 1-Wire support for the temperature sensor, sources the workspace and launches both nodes.

## ⚠️ Limitations

- ⚡ **Peltier cooling is wildly inefficient** — Most energy turns into heat, not cold.  
- 🤖 **ROS2 is overkill** — A full robotic middleware stack for toggling relays is unnecessary.
- 🧯 **No built-in safety features** — No thermal cutoff, current limiting, or fault detection — just vibes and voltage.

## 📈 Potential Improvements

- ❄️ **Switch to a more efficient cooling method** — Just use an ice bucket.
- 🧠 **Replace ROS2 with a microcontroller** — An ESP32 or similar could handle the logic with less power, less latency, and fewer launch files.
- 🔒 **Add safety and protection features** — Implement thermal cutoffs, current sensing, and watchdog timers to prevent electrical regret.

