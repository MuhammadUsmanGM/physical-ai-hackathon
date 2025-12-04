import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import styles from './auth.module.css';

export default function Auth() {
  const [isLogin, setIsLogin] = useState(true);
  const { login, signup } = useAuth();
  const history = useHistory();
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  // Form states
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [softwareBg, setSoftwareBg] = useState('beginner');
  const [hardwareBg, setHardwareBg] = useState('beginner');

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      if (isLogin) {
        await login(email, password);
      } else {
        await signup({
          email,
          password,
          name,
          software_background: softwareBg,
          hardware_background: hardwareBg
        });
      }
      history.push('/');
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Authentication" description="Login or Signup">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <div className={styles.authHeader}>
            <img src="/img/logo.png" alt="Logo" className={styles.logo} />
            <h2>{isLogin ? 'Welcome Back' : 'Start Your Journey'}</h2>
            <p>{isLogin ? 'Login to continue learning' : 'Create an account to personalize your experience'}</p>
          </div>

          {error && <div className={styles.error}>{error}</div>}

          <form onSubmit={handleSubmit} className={styles.authForm}>
            {!isLogin && (
              <div className={styles.formGroup}>
                <label>Full Name</label>
                <input
                  type="text"
                  value={name}
                  onChange={(e) => setName(e.target.value)}
                  required
                  placeholder="John Doe"
                />
              </div>
            )}

            <div className={styles.formGroup}>
              <label>Email Address</label>
              <input
                type="email"
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                required
                placeholder="john@example.com"
              />
            </div>

            <div className={styles.formGroup}>
              <label>Password</label>
              <input
                type="password"
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                required
                placeholder="••••••••"
              />
            </div>

            {!isLogin && (
              <>
                <div className={styles.formGroup}>
                  <label>Software Experience</label>
                  <select value={softwareBg} onChange={(e) => setSoftwareBg(e.target.value)}>
                    <option value="beginner">Beginner (New to coding)</option>
                    <option value="intermediate">Intermediate (Familiar with Python/C++)</option>
                    <option value="advanced">Advanced (Professional Developer)</option>
                  </select>
                </div>

                <div className={styles.formGroup}>
                  <label>Hardware/Robotics Experience</label>
                  <select value={hardwareBg} onChange={(e) => setHardwareBg(e.target.value)}>
                    <option value="beginner">Beginner (No hardware exp)</option>
                    <option value="intermediate">Intermediate (Arduino/Raspberry Pi)</option>
                    <option value="advanced">Advanced (ROS/Industrial Robots)</option>
                  </select>
                </div>
              </>
            )}

            <button type="submit" className={styles.submitButton} disabled={loading}>
              {loading ? 'Processing...' : (isLogin ? 'Login' : 'Create Account')}
            </button>
          </form>

          <div className={styles.authFooter}>
            <p>
              {isLogin ? "Don't have an account? " : "Already have an account? "}
              <button
                className={styles.toggleButton}
                onClick={() => setIsLogin(!isLogin)}
              >
                {isLogin ? 'Sign Up' : 'Login'}
              </button>
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}
