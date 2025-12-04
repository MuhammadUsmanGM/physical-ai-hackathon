import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import styles from './onboarding.module.css';

const PROGRAMMING_LANGUAGES = [
  { id: 'python', name: 'Python', icon: '🐍' },
  { id: 'cpp', name: 'C++', icon: '⚙️' },
  { id: 'javascript', name: 'JavaScript', icon: '🟨' },
  { id: 'java', name: 'Java', icon: '☕' },
  { id: 'ros', name: 'ROS/ROS2', icon: '🤖' },
  { id: 'matlab', name: 'MATLAB', icon: '📊' },
  { id: 'rust', name: 'Rust', icon: '🦀' },
  { id: 'go', name: 'Go', icon: '🔵' }
];

const HARDWARE_CATEGORIES = [
  { id: 'arduino', name: 'Arduino', icon: '🔌' },
  { id: 'raspberry_pi', name: 'Raspberry Pi', icon: '🥧' },
  { id: 'jetson', name: 'NVIDIA Jetson', icon: '💚' },
  { id: 'microcontrollers', name: 'Microcontrollers', icon: '🔧' },
  { id: 'industrial_robots', name: 'Industrial Robots', icon: '🏭' },
  { id: 'mobile_robots', name: 'Mobile Robots', icon: '🚗' },
  { id: 'drones', name: 'Drones/UAVs', icon: '🚁' },
  { id: 'sensors', name: 'Sensors & Actuators', icon: '📡' }
];

const SKILL_LEVELS = [
  { id: 'beginner', name: 'Beginner', description: 'Just starting out' },
  { id: 'intermediate', name: 'Intermediate', description: 'Some experience' },
  { id: 'advanced', name: 'Advanced', description: 'Quite experienced' },
  { id: 'expert', name: 'Expert', description: 'Professional level' }
];

export default function Onboarding() {
  const { user } = useAuth();
  const history = useHistory();
  const [step, setStep] = useState(1);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  // Step 1: Programming Languages
  const [programmingLanguages, setProgrammingLanguages] = useState({});

  // Step 2: Hardware Experience
  const [hardwareExperience, setHardwareExperience] = useState({});

  const handleLanguageSelect = (langId, level) => {
    setProgrammingLanguages(prev => {
      if (prev[langId] === level) {
        const newState = { ...prev };
        delete newState[langId];
        return newState;
      }
      return { ...prev, [langId]: level };
    });
  };

  const handleHardwareSelect = (hwId, level) => {
    setHardwareExperience(prev => {
      if (prev[hwId] === level) {
        const newState = { ...prev };
        delete newState[hwId];
        return newState;
      }
      return { ...prev, [hwId]: level };
    });
  };

  const handleNext = () => {
    if (step === 1 && Object.keys(programmingLanguages).length === 0) {
      setError('Please select at least one programming language');
      return;
    }
    setError('');
    setStep(2);
  };

  const handleBack = () => {
    setError('');
    setStep(1);
  };

  const handleSubmit = async () => {
    if (Object.keys(hardwareExperience).length === 0) {
      setError('Please select at least one hardware category');
      return;
    }

    setLoading(true);
    setError('');

    try {
      const token = localStorage.getItem('token');
      const response = await fetch('http://localhost:8000/auth/onboarding', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify({
          programming_languages: programmingLanguages,
          hardware_experience: hardwareExperience
        })
      });

      if (!response.ok) {
        throw new Error('Failed to complete onboarding');
      }

      history.push('/');
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  const handleSkip = () => {
    history.push('/');
  };

  return (
    <Layout title="Complete Your Profile" description="Tell us about your experience">
      <div className={styles.onboardingContainer}>
        <div className={styles.progressBar}>
          <div className={styles.progressFill} style={{ width: `${(step / 2) * 100}%` }} />
        </div>

        <div className={styles.onboardingCard}>
          <div className={styles.stepIndicator}>
            Step {step} of 2
          </div>

          {step === 1 ? (
            <>
              <div className={styles.header}>
                <h1>What programming languages do you know?</h1>
                <p>Select all that apply and indicate your proficiency level</p>
              </div>

              {error && <div className={styles.error}>{error}</div>}

              <div className={styles.languageGrid}>
                {PROGRAMMING_LANGUAGES.map(lang => (
                  <div key={lang.id} className={styles.languageCard}>
                    <div className={styles.languageHeader}>
                      <span className={styles.languageIcon}>{lang.icon}</span>
                      <span className={styles.languageName}>{lang.name}</span>
                    </div>
                    <div className={styles.skillButtons}>
                      {SKILL_LEVELS.map(level => (
                        <button
                          key={level.id}
                          type="button"
                          className={`${styles.skillButton} ${
                            programmingLanguages[lang.id] === level.id ? styles.selected : ''
                          }`}
                          onClick={() => handleLanguageSelect(lang.id, level.id)}
                          title={level.description}
                        >
                          {level.name}
                        </button>
                      ))}
                    </div>
                  </div>
                ))}
              </div>

              <div className={styles.actions}>
                <button onClick={handleSkip} className={styles.skipButton}>
                  Skip for now
                </button>
                <button onClick={handleNext} className={styles.nextButton}>
                  Next Step →
                </button>
              </div>
            </>
          ) : (
            <>
              <div className={styles.header}>
                <h1>What's your hardware experience?</h1>
                <p>Tell us about your hands-on robotics and hardware experience</p>
              </div>

              {error && <div className={styles.error}>{error}</div>}

              <div className={styles.languageGrid}>
                {HARDWARE_CATEGORIES.map(hw => (
                  <div key={hw.id} className={styles.languageCard}>
                    <div className={styles.languageHeader}>
                      <span className={styles.languageIcon}>{hw.icon}</span>
                      <span className={styles.languageName}>{hw.name}</span>
                    </div>
                    <div className={styles.skillButtons}>
                      {SKILL_LEVELS.map(level => (
                        <button
                          key={level.id}
                          type="button"
                          className={`${styles.skillButton} ${
                            hardwareExperience[hw.id] === level.id ? styles.selected : ''
                          }`}
                          onClick={() => handleHardwareSelect(hw.id, level.id)}
                          title={level.description}
                        >
                          {level.name}
                        </button>
                      ))}
                    </div>
                  </div>
                ))}
              </div>

              <div className={styles.actions}>
                <button onClick={handleBack} className={styles.backButton}>
                  ← Back
                </button>
                <button 
                  onClick={handleSubmit} 
                  className={styles.submitButton}
                  disabled={loading}
                >
                  {loading ? 'Completing...' : 'Complete Profile'}
                </button>
              </div>
            </>
          )}
        </div>
      </div>
    </Layout>
  );
}
