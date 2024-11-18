# ros_voice_assistant

## Overview

**ros_voice_assistant** is a ROS 2 package designed for voice interaction, utilizing Google Text-to-Speech (TTS) and SpeechRecognition to provide a seamless voice assistant experience. It allows users to interact with a robot or system using predefined keywords and phrases while providing customizable responses.

The assistant is configurable, allowing users to change the **keyword** it listens for and the **language** it recognizes. Additionally, it comes with built-in support for a variety of **commands** that users can edit to suit their needs.

## Disclaimer

This package uses the **Google Text-to-Speech (TTS) service**. By using this package, you agree to Google’s Terms of Service and Privacy Policy. Please review these terms before use.

## License

The source code is released under a [MIT License](./LICENSE).

**Author**: Victor Carvalho<br />

## Key Features

- **Keyword-based Activation:** The assistant activates when it hears the predefined keyword (default: `"Chloe"`). You can configure the keyword to suit your needs.
- **Predefined Phrases:** Responds to a set of built-in commands, which are fully customizable.
- **Multilingual Support:** Supports different languages via the `language` parameter (default: `"en-US"`).
- **Extensibility:** Add or modify commands and responses easily within the `nlp_node.py`.

## Applications

The **ros_voice_assistant** package is versatile and can be used in various voice-interaction scenarios for robotics and automation. Here are some example applications:

1. **Personal Assistant for Robots:**
   - Add conversational capabilities to robots, enabling them to respond to questions, perform tasks, or provide feedback to users.

2. **Voice-Controlled Home Automation:**
   - Control smart devices or home appliances through voice commands, using a robot as the central interface.

3. **Interactive Conversational Agents:**
   - Use the assistant as a chatbot-like interface for entertainment, FAQs, or providing user guidance.

4. **Telepresence Robots:**
   - Enable natural language interactions during remote communication via telepresence robots.

5. **Education and Research:**
   - A great tool for teaching robotics and experimenting with voice-enabled human-robot interaction (HRI).

6. **Accessibility Tools:**
   - Help users with disabilities control robots or devices through voice commands.

7. **Voice-Controlled Navigation:**
   - Direct mobile robots to specific locations using simple voice commands like "go to the kitchen" or "return to base."

These applications demonstrate the flexibility of the **ros_voice_assistant** and its potential to enhance human-robot interaction in diverse environments.

## Predefined Commands and Responses

Below are the default phrases the assistant understands. These can be edited in the **`nlp_node.py`** file.

| **Command**                              | **Default Response**                                                                  |
|------------------------------------------|---------------------------------------------------------------------------------------|
| **"hello"**, **"hi"**, **"hey"**         | `"Hey there! How's it going today?"`                                                  |
| **"how are you"**, **"how's it going"**  | `"I'm just a bot, but I'm feeling pretty good! Thanks for asking. How about you?"`    |
| **"tell me a joke"**, **"make me laugh"**| `"Why did the robot go on vacation? Because it needed to recharge its batteries!"`    |
| **"what's the weather"**                 | `"I wish I could check the weather for you, but my sensors aren't that advanced yet!"`|
| **"what can you do"**                    | `"I can chat with you, tell jokes, and maybe even learn new things if you teach me!"` |
| **"goodbye"**, **"bye"**                 | `"Goodbye! It was nice chatting with you. Come back soon!"`                           |
| **"who are you"**, **"what are you"**    | `"I'm your friendly chatbot! My job is to keep you entertained and assist you."`      |
| **Unknown Phrase**                       | `"Hmm, I'm not sure I understand. Could you repeat that, please?"`                    |

## Parameters

You can configure the behavior of the assistant by setting the following parameters in the **launch file** or during runtime.

### **MicNode Parameters**
| **Parameter** | **Type**  | **Description**                                        | **Default**  |
|---------------|-----------|--------------------------------------------------------|--------------|
| `language`    | `string`  | Language code for speech recognition (`en-US`, etc.).  | `"en-US"`    |
| `keyword`     | `string`  | The word that activates the assistant.                 | `"Chloe"`    |

### **SpeakerNode Parameters**
| **Parameter** | **Type** | **Description**                                    | **Default** |
|---------------|----------|----------------------------------------------------|-------------|
| `sync_mode`   | `bool`   | If `True`, blocks until the speech finishes.       | `True`      |

## Package Main Files

```
ros_voice_assistant/
├── voice_assistant/
│   ├── launch/
│   │   └── voice_assistant.launch.py
│   ├── voice_assistant/
│   │   ├── __init__.py
│   │   ├── mic_node.py
│   │   ├── nlp_node.py
│   │   └── speaker_node.py
│   ├── LICENSE
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── assistant_interfaces/
│   ├── srv/
│   │   └── Text.srv
│   ├── CMakeLists.txt
│   ├── LICENSE
│   └── package.xml
├── LICENSE
├── README.md
└── requirements.txt
```

## Installation and Setup

1. **Clone this repository** into your ROS 2 workspace:
```bash
mkdir -p project_ws/src && cd project_ws  # If you don't have a workspace
git clone https://github.com/victorcarvesk/ros-voice-assistant.git src/ros_voice_assistant
```

1. **Install package system dependencies** with rosdep:
```bash
rosdep update
rosdep install --from-paths src/ros_voice_assistant -y --ignore-src
```

1. **Create and activate a virtual environment** (required on Ubuntu 24.04+):
```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
```

1. **Install package Python dependencies**:
```bash
pip3 install -U -r src/ros_voice_assistant/requirements.txt
```

1. **Build the package** with colcon:
```bash
colcon build --packages-select assistant_interfaces voice_assistant
```

1. **Launch the voice assistant**:
```bash
. install/setup.bash
ros2 launch voice_assistant voice_assistant.launch.py
```

1. **Talk to your assistant**:

Try commands like:
- **"Hello, Chloe!"**
- **"Chloe, tell me a joke."**
- **"Chloe, what can you do?"**

## Customizing the Assistant

### **Changing the Activation Keyword**
Modify the `keyword` parameter in the launch file:
```python
Node(
    package='voice_assistant',
    executable='mic_node',
    output='screen',
    parameters=[
        {'language': 'en-US'},
        {'keyword': 'Chloe'},  # Change the keyword here
    ],
)
```

### **Editing Commands and Responses**
The predefined commands and responses are defined in `nlp_node.py`. You can modify or extend these by editing the `nlp_callback` method.

Example:
```python
case "play music":
    answer = "Sure! What would you like me to play?"
```

## Feedback and Contributions

Feel free to open issues or submit pull requests to improve this package. Contributions are welcome!
