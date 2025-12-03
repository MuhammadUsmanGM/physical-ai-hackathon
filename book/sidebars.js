module.exports = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 01: Introduction to Physical AI',
      items: [
        {
          type: 'doc',
          id: '01-Introduction-to-Physical-AI/index',
          label: 'Module Overview'
        },
        {
          type: 'category',
          label: 'Core Concepts',
          items: [
            {
              type: 'doc',
              id: '01-Introduction-to-Physical-AI/01-What-is-Physical-AI/index',
              label: 'What is Physical AI?'
            },
            {
              type: 'doc',
              id: '01-Introduction-to-Physical-AI/02-Why-Physical-AI-Matters/index',
              label: 'Why Physical AI Matters'
            },
            {
              type: 'doc',
              id: '01-Introduction-to-Physical-AI/03-Sensor-Systems/index',
              label: 'Sensor Systems Deep Dive'
            },
            {
              type: 'doc',
              id: '01-Introduction-to-Physical-AI/04-Embodiment-Physical-Reasoning/index',
              label: 'Embodiment & Physical Reasoning'
            },
            {
              type: 'doc',
              id: '01-Introduction-to-Physical-AI/05-Applications-Case-Studies/index',
              label: 'Real-World Applications & Case Studies'
            }
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 02: The Robotic Nervous System (ROS 2)',
      items: [
        {
          type: 'doc',
          id: '02-The-Robotic-Nervous-System/index',
          label: 'Module Overview'
        },
        {
          type: 'category',
          label: 'ROS 2 Fundamentals',
          items: [
            {
              type: 'doc',
              id: '02-The-Robotic-Nervous-System/01-ROS2-Fundamentals/index',
              label: 'ROS 2 Fundamentals'
            },
            {
              type: 'doc',
              id: '02-The-Robotic-Nervous-System/02-ROS2-Environment-Setup/index',
              label: 'Environment Setup'
            },
            {
              type: 'doc',
              id: '02-The-Robotic-Nervous-System/03-ROS2-Communication-Patterns/index',
              label: 'Communication Patterns Deep Dive'
            }
          ]
        },
        {
          type: 'category',
          label: 'Robot Description & Packages',
          items: [
            {
              type: 'doc',
              id: '02-The-Robotic-Nervous-System/04-URDF-for-Humanoids/index',
              label: 'URDF for Humanoids'
            },
            {
              type: 'doc',
              id: '02-The-Robotic-Nervous-System/05-Building-ROS2-Packages/index',
              label: 'Building ROS 2 Packages'
            },
            {
              type: 'doc',
              id: '02-The-Robotic-Nervous-System/06-Launch-Files-Parameters/index',
              label: 'Launch Files & Parameters'
            }
          ]
        },
        {
          type: 'category',
          label: 'AI Integration',
          items: [
            {
              type: 'doc',
              id: '02-The-Robotic-Nervous-System/07-Python-Agents-ROS-Bridge/index',
              label: 'Bridging Python AI Agents to ROS'
            }
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 03: The Digital Twin (Gazebo & Unity)',
      items: [
        {
          type: 'doc',
          id: '03-The-Digital-Twin/index',
          label: 'Module Overview'
        },
        {
          type: 'category',
          label: 'Simulation Platforms',
          items: [
            {
              type: 'doc',
              id: '03-The-Digital-Twin/01-Gazebo-Simulation/index',
              label: 'Gazebo Simulation'
            },
            {
              type: 'doc',
              id: '03-The-Digital-Twin/02-Unity-Simulation/index',
              label: 'Unity Simulation'
            }
          ]
        },
        {
          type: 'category',
          label: 'Advanced Topics',
          items: [
            {
              type: 'doc',
              id: '03-The-Digital-Twin/03-URDF-SDF-Deep-Dive/index',
              label: 'URDF and SDF (In-Depth)'
            },
            {
              type: 'doc',
              id: '03-The-Digital-Twin/04-Physics-Simulation/index',
              label: 'Physics Simulation Fundamentals'
            },
            {
              type: 'doc',
              id: '03-The-Digital-Twin/05-Domain-Randomization/index',
              label: 'Domain Randomization Techniques'
            },
            {
              type: 'doc',
              id: '03-The-Digital-Twin/06-Sim-to-Real-Transfer/index',
              label: 'Sim-to-Real Transfer'
            }
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 04: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        {
          type: 'doc',
          id: '04-The-AI-Robot-Brain/index',
          label: 'Module Overview'
        },
        {
          type: 'category',
          label: 'Isaac Platform',
          items: [
            {
              type: 'doc',
              id: '04-The-AI-Robot-Brain/01-NVIDIA-Isaac-Sim/index',
              label: 'Isaac Sim'
            },
            {
              type: 'doc',
              id: '04-The-AI-Robot-Brain/02-Isaac-ROS-Perception/index',
              label: 'Isaac ROS Perception'
            },
            {
              type: 'doc',
              id: '04-The-AI-Robot-Brain/03-Nav2-Bipedal-Navigation/index',
              label: 'Nav2 for Bipedal Navigation'
            }
          ]
        },
        {
          type: 'category',
          label: 'AI & Deployment',
          items: [
            {
              type: 'doc',
              id: '04-The-AI-Robot-Brain/04-RL-Robot-Control/index',
              label: 'Reinforcement Learning for Robot Control'
            },
            {
              type: 'doc',
              id: '04-The-AI-Robot-Brain/05-Jetson-Deployment/index',
              label: 'Deploying to Jetson Hardware'
            }
          ]
        }
      ]
    },
    {
      type: 'category',
      label: 'Module 05: Vision-Language-Action (VLA)',
      items: [
        {
          type: 'doc',
          id: '05-Vision-Language-Action/index',
          label: 'Module Overview'
        },
        {
          type: 'category',
          label: 'VLA Components',
          items: [
            {
              type: 'doc',
              id: '05-Vision-Language-Action/01-Voice-to-Action-Systems/index',
              label: 'Voice to Action Systems'
            },
            {
              type: 'doc',
              id: '05-Vision-Language-Action/03-Cognitive-Planning-LLMs/index',
              label: 'Cognitive Planning with LLMs'
            },
            {
              type: 'doc',
              id: '05-Vision-Language-Action/04-Multi-Modal-Interaction/index',
              label: 'Multi-Modal Interaction'
            },
            {
              type: 'doc',
              id: '05-Vision-Language-Action/05-VLA-Implementation/index',
              label: 'End-to-End VLA Implementation'
            }
          ]
        },
        {
          type: 'category',
          label: 'Capstone',
          items: [
            {
              type: 'doc',
              id: '05-Vision-Language-Action/02-Capstone-Project/index',
              label: 'Capstone Project'
            }
          ]
        }
      ]
    }
  ],
};
