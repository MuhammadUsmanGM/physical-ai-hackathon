import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import styles from './auth.module.css';

import useBaseUrl from '@docusaurus/useBaseUrl';

export default function Auth() {
  const [isLogin, setIsLogin] = useState(true);
  const { login, signup } = useAuth();
  const history = useHistory();
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const logoUrl = useBaseUrl('/img/logo.png');

  // Form states
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');

  // Password visibility toggle
  const [showPassword, setShowPassword] = useState(false);
  const [passwordStrength, setPasswordStrength] = useState(''); // 'weak', 'moderate', 'strong'

  // Password strength checker function
  const checkPasswordStrength = (password) => {
    if (password.length === 0) return '';

    let strength = 0;
    if (password.length >= 8) strength++;
    if (/[A-Z]/.test(password)) strength++;
    if (/[a-z]/.test(password)) strength++;
    if (/[0-9]/.test(password)) strength++;
    if (/[^A-Za-z0-9]/.test(password)) strength++;

    if (strength >= 4) return 'strong';
    if (strength >= 2) return 'moderate';
    return 'weak';
  };

  // Handle password change for strength indicator
  const handlePasswordChange = (e) => {
    const newPassword = e.target.value;
    setPassword(newPassword);

    // Update password strength when on signup page
    if (!isLogin) {
      setPasswordStrength(checkPasswordStrength(newPassword));
    }
  };

  const [successMsg, setSuccessMsg] = useState('');

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setSuccessMsg('');
    setLoading(true);

    try {
      if (isLogin) {
        await login(email, password);
        // Use useBaseUrl to get the correct path for the landing page (handles GitHub Pages subpaths)
        history.push(useBaseUrl('/'));
      } else {
        await signup({
          email,
          password,
          name
        });
        // Instead of redirecting, show success and switch to login
        setSuccessMsg('Signup successful! Please login with your new account.');
        setIsLogin(true);
        setPassword(''); // Clear password for security
      }
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  // Get icon color based on password strength
  const getIconColor = () => {
    if (passwordStrength === 'strong') return '#10B981'; // Green
    if (passwordStrength === 'moderate') return '#F59E0B'; // Yellow
    if (passwordStrength === 'weak') return '#EF4444'; // Red
    return '#9CA3AF'; // Default gray
  };

  return (
    <Layout title="Authentication" description="Login or Signup">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <div className={styles.authHeader}>
            <img src={logoUrl} alt="Logo" className={styles.logo} />
            <h2>{isLogin ? 'Welcome Back' : 'Start Your Journey'}</h2>
            <p>{isLogin ? 'Login to continue learning' : 'Create an account to personalize your experience'}</p>
          </div>

          {error && <div className={styles.error}>{error}</div>}
          {successMsg && <div className="alert alert--success margin-bottom--md">{successMsg}</div>}

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
              <div className={styles.passwordContainer}>
                <input
                  type={showPassword ? 'text' : 'password'}
                  value={password}
                  onChange={handlePasswordChange}
                  required
                  placeholder="••••••••"
                />
                <button
                  type="button"
                  className={styles.eyeButton}
                  style={{ color: isLogin ? '#9CA3AF' : getIconColor() }}
                  onClick={() => setShowPassword(!showPassword)}
                >
                  {showPassword ? (
                    // Eye icon (closed)
                    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                      <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z" />
                      <circle cx="12" cy="12" r="3" />
                      <path d="M12 10v4" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
                      <path d="M10 12h4" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
                    </svg>
                  ) : (
                    // Eye icon (open)
                    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                      <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z" />
                      <circle cx="12" cy="12" r="3" />
                    </svg>
                  )}
                </button>
              </div>
            </div>

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
