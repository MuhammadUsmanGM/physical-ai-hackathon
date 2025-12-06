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
  const [validationErrors, setValidationErrors] = useState({});

  // Password visibility toggle
  const [showPassword, setShowPassword] = useState(false);
  const [passwordStrength, setPasswordStrength] = useState(''); // 'weak', 'moderate', 'strong'

  // Email validation
  const validateEmail = (email) => {
    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    return emailRegex.test(email);
  };

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

  // Validate form inputs
  const validateForm = () => {
    const errors = {};
    
    // Trim inputs
    const trimmedEmail = email.trim();
    const trimmedName = name.trim();
    
    if (!isLogin && trimmedName.length < 2) {
      errors.name = 'Name must be at least 2 characters';
    }
    
    if (!validateEmail(trimmedEmail)) {
      errors.email = 'Please enter a valid email address';
    }
    
    if (password.length < 8) {
      errors.password = 'Password must be at least 8 characters';
    }
    
    if (!isLogin && passwordStrength === 'weak') {
      errors.password = 'Password is too weak. Add uppercase, numbers, and special characters';
    }
    
    setValidationErrors(errors);
    return Object.keys(errors).length === 0;
  };

  // Handle password change for strength indicator
  const handlePasswordChange = (e) => {
    const newPassword = e.target.value;
    setPassword(newPassword);
    setValidationErrors(prev => ({ ...prev, password: '' }));

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
    setValidationErrors({});
    
    // Validate form
    if (!validateForm()) {
      return;
    }
    
    setLoading(true);

    try {
      if (isLogin) {
        await login(email.trim(), password);
        history.push(useBaseUrl('/'));
      } else {
        // Redirect to home page after signup
        await signup({
          email: email.trim(),
          password,
          name: name.trim()
        });
        // Use window.location for full page reload
        window.location.href = useBaseUrl('/');
      }
    } catch (err) {
      // Improved error messages
      const errorMessage = err.message || 'An error occurred';
      
      if (errorMessage.includes('Network') || errorMessage.includes('fetch')) {
        setError('Network error. Please check your internet connection and try again.');
      } else if (errorMessage.includes('already exists') || errorMessage.includes('duplicate')) {
        setError('This email is already registered. Please login or use a different email.');
      } else if (errorMessage.includes('Invalid') || errorMessage.includes('incorrect')) {
        setError('Invalid email or password. Please try again.');
      } else if (errorMessage.includes('500') || errorMessage.includes('server')) {
        setError('Server error. Please try again later.');
      } else {
        setError(errorMessage);
      }
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

          <form onSubmit={handleSubmit} className={styles.authForm}>
            {!isLogin && (
              <div className={styles.formGroup}>
                <label>Full Name</label>
                <input
                  type="text"
                  value={name}
                  onChange={(e) => {
                    setName(e.target.value);
                    setValidationErrors(prev => ({ ...prev, name: '' }));
                  }}
                  required
                  disabled={loading}
                  placeholder="John Doe"
                  className={validationErrors.name ? styles.inputError : ''}
                />
                {validationErrors.name && <span className={styles.fieldError}>{validationErrors.name}</span>}
              </div>
            )}

            <div className={styles.formGroup}>
              <label>Email Address</label>
              <input
                type="email"
                value={email}
                onChange={(e) => {
                  setEmail(e.target.value);
                  setValidationErrors(prev => ({ ...prev, email: '' }));
                }}
                required
                disabled={loading}
                placeholder="john@example.com"
                className={validationErrors.email ? styles.inputError : ''}
              />
              {validationErrors.email && <span className={styles.fieldError}>{validationErrors.email}</span>}
            </div>

            <div className={styles.formGroup}>
              <label>Password</label>
              <div className={styles.passwordContainer}>
                <input
                  type={showPassword ? 'text' : 'password'}
                  value={password}
                  onChange={handlePasswordChange}
                  required
                  disabled={loading}
                  placeholder="••••••••"
                  className={validationErrors.password ? styles.inputError : ''}
                />
                <button
                  type="button"
                  className={styles.eyeButton}
                  style={{ color: isLogin ? '#9CA3AF' : getIconColor() }}
                  onClick={() => setShowPassword(!showPassword)}
                  disabled={loading}
                >
                  {showPassword ? (
                    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                      <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z" />
                      <circle cx="12" cy="12" r="3" />
                      <path d="M12 10v4" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
                      <path d="M10 12h4" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
                    </svg>
                  ) : (
                    <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                      <path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z" />
                      <circle cx="12" cy="12" r="3" />
                    </svg>
                  )}
                </button>
              </div>
              {validationErrors.password && <span className={styles.fieldError}>{validationErrors.password}</span>}
              {!isLogin && password.length > 0 && (
                <div className={styles.passwordRequirements}>
                  <small style={{ color: password.length >= 8 ? '#10B981' : '#9CA3AF' }}>
                    ✓ At least 8 characters
                  </small>
                  <small style={{ color: /[A-Z]/.test(password) ? '#10B981' : '#9CA3AF' }}>
                    ✓ One uppercase letter
                  </small>
                  <small style={{ color: /[0-9]/.test(password) ? '#10B981' : '#9CA3AF' }}>
                    ✓ One number
                  </small>
                  <small style={{ color: /[^A-Za-z0-9]/.test(password) ? '#10B981' : '#9CA3AF' }}>
                    ✓ One special character
                  </small>
                </div>
              )}
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
