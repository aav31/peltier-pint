# 🍺 peltier-pint

An unnecessarily complicated and wildly inefficient way to chill a pint.

Because when you’re holding a perfectly drinkable lager at 10°C, the only reasonable next step is to boot up a full ROS2 stack, engage two relays, and fire 250 watts at it until it submits to 6°C perfection.


![cooling-demo.gif](media/cooling-demo.gif)  
*Above: A surprisingly effective thermodynamic crime in progress.*

---

## 🍻 Why I Made This

This was my way of doing something *fun and functional* with ROS2, beyond turtlesim and generic tutorials. I wanted to combine hardware control, topic communication, and some real-world feedback.

This is not a production-grade beverage cooler. But it is a project that brought together sensors, control loops, and reckless thermoelectric ambition in one satisfying weekend build.

Yes, I could’ve just bought a mini fridge or used an ice bucket. But where’s the fun in that?

---

## 🧊 The Cooling Strategy

- **Target Temperature**: 6–7°C
- **Control Method**: Classic *bang-bang* control — if it's too warm, the system turns on the cooling hardware; if it's cold enough, it turns it off.
- **Hardware**:
  - 4x Peltier modules
  - 4x Cooling fans
  - 2x Relays switching a 12V / 250W PSU
  - 1x DS18B20 sensor for temperature feedback
  - 1x Raspberry Pi 4
- **Software**: 2 ROS2 nodes — one publishes the temperature, the other controls the cooling.

---

## 📂 Files & Media

- **[temperature_publisher.py](src/peltier_pint/temperature_publisher.py)** – Reads from the DS18B20 sensor and publishes data to the `/temperature` topic.
- **[relay_control_sensor.py](src/peltier_pint/relay_control_sensor.py)** – Subscribes to `/temperature`, then turns the relays on/off depending on whether the reading is within range.
- **[start_cooler.sh](start_cooler.sh)** – Startup script which: enables 1-Wire support for the temperature sensor, sources the workspace and launches both nodes.

---

## 🪛 Limitations

- ❄️ Cooling is slow and inefficient. Like... really inefficient. But it *does* work.  
- ⚡ 250W is arguably too much to cool a single pint. Especially when half of it goes to waste heat.  
- 🤖 ROS2 and a Raspberry Pi is overkill for this.

---

## 📈 Potential Improvements

- Use PID control instead of bang-bang for smoother operation (and fewer relay clicks).  
- Log temperature over time with `rqt_plot` or a custom node.
- Add a web dashboard or display to show current status.
- Make it multi-pint compatible (party mode?).
- Rebuild in a tidy enclosure and publish proper wiring diagrams.

---
