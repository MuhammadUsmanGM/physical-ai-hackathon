import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './onboarding.module.css';

// Software technologies relevant to Physical AI & Robotics
const SOFTWARE_OPTIONS = [
  { id: 'python', name: 'Python', icon: '🐍', category: 'Language' },
  { id: 'cpp', name: 'C++', icon: '⚡', category: 'Language' },
  { id: 'javascript', name: 'JavaScript', icon: '💛', category: 'Language' },
  { id: 'ros2', name: 'ROS 2', icon: '🤖', category: 'Framework' },
  { id: 'pytorch', name: 'PyTorch', icon: '🔥', category: 'AI Framework' },
  { id: 'tensorflow', name: 'TensorFlow', icon: '🧠', category: 'AI Framework' },
  { id: 'opencv', name: 'OpenCV', icon: '👁️', category: 'Computer Vision' },
  { id: 'gazebo', name: 'Gazebo', icon: '🌐', category: 'Simulation' },
];

// Hardware platforms relevant to Physical AI & Robotics
const HARDWARE_OPTIONS = [
  { id: 'jetson', name: 'NVIDIA Jetson', icon: '🟢', category: 'Edge AI' },
  { id: 'raspberry_pi', name: 'Raspberry Pi', icon: '🥧', category: 'SBC' },
  { id: 'arduino', name: 'Arduino', icon: '🔵', category: 'Microcontroller' },
  { id: 'realsense', name: 'Intel RealSense', icon: '📷', category: 'Camera' },
  { id: 'lidar', name: 'LiDAR Sensors', icon: '📡', category: 'Sensor' },
  { id: 'isaac_sim', name: 'NVIDIA Isaac Sim', icon: '🎮', category: 'Simulation' },
  { id: 'unitree', name: 'Unitree Robots', icon: '🦾', category: 'Robot Platform' },
  { id: 'robotic_arm', name: 'Robotic Arms', icon: '🦿', category: 'Robot Platform' },
];

const PROFICIENCY_LEVELS = ['Beginner', 'Intermediate', 'Advanced', 'Expert'];

const EXPERIENCE_LEVELS = [
  {
    id: 'beginner',
    title: 'Beginner',
    icon: '🌱',
    description: 'New to robotics and AI. Excited to learn the fundamentals!'
  },
  {
    id: 'intermediate',
    title: 'Intermediate',
    icon: '🚀',
    description: 'Some experience with programming or robotics. Ready to dive deeper!'
  },
  {
    id: 'advanced',
    title: 'Advanced',
    icon: '⚡',
    description: 'Experienced with AI/robotics. Looking to master Physical AI!'
  }
];

export default function Onboarding() {
  const { completeOnboarding, user, loading: authLoading } = useAuth();
  const history = useHistory();
  const [currentStep, setCurrentStep] = useState(1);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  // Ref to track if component is mounted to prevent state updates on unmounted components
  const isMountedRef = React.useRef(true);

  // Form state
  const [softwareExperience, setSoftwareExperience] = useState({});
  const [hardwareExperience, setHardwareExperience] = useState({});
  const [experienceLevel, setExperienceLevel] = useState('');

  // Cleanup function to set mounted ref to false on unmount
  React.useEffect(() => {
    return () => {
      isMountedRef.current = false;
    };
  }, []);

  // All event handlers MUST be defined before any conditional returns
  const handleSoftwareToggle = (techId) => {
    setSoftwareExperience(prev => {
      const newExp = { ...prev };
      if (newExp[techId]) {
        delete newExp[techId];
      } else {
        newExp[techId] = 'Beginner';
      }
      return newExp;
    });
  };

  const handleSoftwareLevel = (techId, level) => {
    setSoftwareExperience(prev => ({
      ...prev,
      [techId]: level
    }));
  };

  const handleHardwareToggle = (hardwareId) => {
    setHardwareExperience(prev => {
      const newExp = { ...prev };
      if (newExp[hardwareId]) {
        delete newExp[hardwareId];
      } else {
        newExp[hardwareId] = 'Beginner';
      }
      return newExp;
    });
  };

  const handleHardwareLevel = (hardwareId, level) => {
    setHardwareExperience(prev => ({
      ...prev,
      [hardwareId]: level
    }));
  };

  const handleNext = () => {
    if (currentStep < 3) {
      setCurrentStep(currentStep + 1);
    }
  };

  const handleBack = () => {
    if (currentStep > 1) {
      setCurrentStep(currentStep - 1);
    }
  };

  const handleSkipStep = () => {
    if (currentStep < 3) {
      setCurrentStep(currentStep + 1);
    } else {
      handleSubmit();
    }
  };

  const handleSubmit = async () => {
    if (!experienceLevel) {
      setError('Please select your overall experience level');
      return;
    }

    setLoading(true);
    setError('');

    try {
      await completeOnboarding({
        software_experience: softwareExperience,
        hardware_experience: hardwareExperience,
        experience_level: experienceLevel,
        areas_of_interest: []
      });

      // Only redirect if component is still mounted
      if (isMountedRef.current) {
        // Redirect to home page
        history.push(useBaseUrl('/'));
      }
    } catch (err) {
      // Only set error if component is still mounted
      if (isMountedRef.current) {
        setError(err.message || 'Failed to complete onboarding');
      }
    } finally {
      // Only update loading state if component is still mounted
      if (isMountedRef.current) {
        setLoading(false);
      }
    }
  };

  // ALL RENDER FUNCTIONS MUST BE BEFORE CONDITIONAL RETURNS
  const renderProgressBar = () => (
    <div className={styles.progressBar}>
      {[1, 2, 3].map(step => (
        <div key={step} className={styles.progressStep}>
          <div className={`${styles.stepCircle} ${currentStep === step ? styles.active : ''} ${currentStep > step ? styles.completed : ''}`}>
            {currentStep > step ? '✓' : step}
          </div>
          <span className={`${styles.stepLabel} ${currentStep === step ? styles.active : ''}`}>
            {step === 1 ? 'Software' : step === 2 ? 'Hardware' : 'Experience'}
          </span>
        </div>
      ))}
    </div>
  );

  const renderStep1 = () => (
    <div className={styles.stepContent}>
      <h2 className={styles.stepTitle}>Software Experience 💻</h2>
      <p className={styles.stepDescription}>
        Select the programming languages and frameworks you're familiar with, and indicate your proficiency level.
      </p>

      <div className={styles.selectionGrid}>
        {SOFTWARE_OPTIONS.map(tech => (
          <div key={tech.id}>
            <div
              className={`${styles.selectionCard} ${softwareExperience[tech.id] ? styles.selected : ''}`}
              onClick={() => handleSoftwareToggle(tech.id)}
            >
              <div className={styles.cardIcon}>{tech.icon}</div>
              <div className={styles.cardTitle}>{tech.name}</div>
              <div className={styles.cardSubtitle}>{tech.category}</div>
            </div>
            {softwareExperience[tech.id] && (
              <div className={styles.levelSelector}>
                {PROFICIENCY_LEVELS.map(level => (
                  <button
                    key={level}
                    className={`${styles.levelButton} ${softwareExperience[tech.id] === level ? styles.selected : ''}`}
                    onClick={(e) => {
                      e.stopPropagation();
                      handleSoftwareLevel(tech.id, level);
                    }}
                  >
                    {level}
                  </button>
                ))}
              </div>
            )}
          </div>
        ))}
      </div>

      <button className={styles.skipButton} onClick={handleSkipStep}>
        Skip this step →
      </button>
    </div>
  );

  const renderStep2 = () => (
    <div className={styles.stepContent}>
      <h2 className={styles.stepTitle}>Hardware Experience 🔧</h2>
      <p className={styles.stepDescription}>
        Select the hardware platforms and robotics tools you've worked with, and indicate your experience level.
      </p>

      <div className={styles.selectionGrid}>
        {HARDWARE_OPTIONS.map(hardware => (
          <div key={hardware.id}>
            <div
              className={`${styles.selectionCard} ${hardwareExperience[hardware.id] ? styles.selected : ''}`}
              onClick={() => handleHardwareToggle(hardware.id)}
            >
              <div className={styles.cardIcon}>{hardware.icon}</div>
              <div className={styles.cardTitle}>{hardware.name}</div>
              <div className={styles.cardSubtitle}>{hardware.category}</div>
            </div>
            {hardwareExperience[hardware.id] && (
              <div className={styles.levelSelector}>
                {PROFICIENCY_LEVELS.map(level => (
                  <button
                    key={level}
                    className={`${styles.levelButton} ${hardwareExperience[hardware.id] === level ? styles.selected : ''}`}
                    onClick={(e) => {
                      e.stopPropagation();
                      handleHardwareLevel(hardware.id, level);
                    }}
                  >
                    {level}
                  </button>
                ))}
              </div>
            )}
          </div>
        ))}
      </div>

      <button className={styles.skipButton} onClick={handleSkipStep}>
        Skip this step →
      </button>
    </div>
  );

  const renderStep3 = () => (
    <div className={styles.stepContent}>
      <h2 className={styles.stepTitle}>Overall Experience Level 🎯</h2>
      <p className={styles.stepDescription}>
        How would you describe your overall experience with Physical AI and Robotics?
      </p>

      <div className={styles.experienceLevelGrid}>
        {EXPERIENCE_LEVELS.map(level => (
          <div
            key={level.id}
            className={`${styles.experienceLevelCard} ${experienceLevel === level.id ? styles.selected : ''}`}
            onClick={() => setExperienceLevel(level.id)}
          >
            <div className={styles.experienceLevelIcon}>{level.icon}</div>
            <div className={styles.experienceLevelTitle}>{level.title}</div>
            <div className={styles.experienceLevelDescription}>{level.description}</div>
          </div>
        ))}
      </div>
    </div>
  );

  // Redirect if user is not logged in or already completed onboarding
  React.useEffect(() => {
    if (authLoading) return; // Wait for auth to load

    if (!user) {
      // Not logged in, redirect to auth page
      history.push(useBaseUrl('/auth'));
      return;
    }

    if (user.onboarding_completed) {
      // Already completed onboarding, redirect to home
      history.push(useBaseUrl('/'));
      return;
    }
  }, [user, authLoading, history]);

  // Show loading while auth is loading
  if (authLoading) {
    return (
      <Layout title="Loading..." description="Loading">
        <div className={styles.onboardingContainer}>
          <div className={styles.onboardingCard}>
            <div className={styles.header}>
              <h1>Loading... ⏳</h1>
              <p>Please wait while we prepare your onboarding experience</p>
            </div>
          </div>
        </div>
      </Layout>
    );
  }

  // Don't render if user is not logged in or already completed
  if (!user) {
    return null; // Don't render anything if user is not loaded yet
  }

  // If onboarding is already completed, don't render anything
  // The useEffect handles the redirect
  if (user.onboarding_completed) {
    return null;
  }

  return (
    <Layout title="Welcome Onboarding" description="Complete your profile">
      <div className={styles.onboardingContainer}>
        <div className={styles.onboardingCard}>
          <div className={styles.header}>
            <h1>Welcome, {user?.name}! 👋</h1>
            <p>Let's personalize your learning experience</p>
          </div>

          {renderProgressBar()}

          {error && <div className={styles.error}>{error}</div>}

          {currentStep === 1 && renderStep1()}
          {currentStep === 2 && renderStep2()}
          {currentStep === 3 && renderStep3()}

          <div className={styles.navigationButtons}>
            {currentStep > 1 && (
              <button
                className={`${styles.button} ${styles.buttonSecondary}`}
                onClick={handleBack}
                disabled={loading}
              >
                ← Back
              </button>
            )}
            {currentStep < 3 ? (
              <button
                className={`${styles.button} ${styles.buttonPrimary}`}
                onClick={handleNext}
                disabled={loading}
              >
                Next →
              </button>
            ) : (
              <button
                className={`${styles.button} ${styles.buttonPrimary}`}
                onClick={handleSubmit}
                disabled={loading || !experienceLevel}
              >
                {loading ? 'Completing...' : 'Complete Setup 🚀'}
              </button>
            )}
          </div>
        </div>
      </div>
    </Layout>
  );
}
