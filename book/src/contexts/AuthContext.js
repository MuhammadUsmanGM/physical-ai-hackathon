import React, { createContext, useContext, useState, useEffect } from 'react';
import { useHistory } from '@docusaurus/router';

const AuthContext = createContext(null);

// const API_URL = 'http://localhost:4000'; // Local
const API_URL = 'https://physical-ai-hackathon.vercel.app'; // Production

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [token, setToken] = useState(null);
  const history = useHistory();

  useEffect(() => {
    // Check for token in local storage on mount
    const storedToken = localStorage.getItem('auth_token');
    if (storedToken) {
      // Check if token is expired before using it
      if (isTokenExpired(storedToken)) {
        logout();
        setLoading(false);
      } else {
        setToken(storedToken);
        checkSession(storedToken);
      }
    } else {
      setLoading(false);
    }
  }, []);

  // Check if JWT token is expired
  const isTokenExpired = (token) => {
    try {
      const payload = JSON.parse(atob(token.split('.')[1]));
      // Check if token has expiration and if it's expired
      if (payload.exp) {
        return payload.exp * 1000 < Date.now();
      }
      return false;
    } catch (error) {
      console.error('Error parsing token:', error);
      return true; // If we can't parse it, consider it expired
    }
  };

  const checkSession = async (authToken) => {
    try {
      const response = await fetch(`${API_URL}/api/auth/me`, {
        headers: {
          'Authorization': `Bearer ${authToken}`
        }
      });

      if (response.ok) {
        const data = await response.json();
        setUser(data.user);
      } else {
        // Token invalid
        logout();
      }
    } catch (error) {
      console.error('Session check failed', error);
      logout();
    } finally {
      setLoading(false);
    }
  };

  const login = async (email, password) => {
    try {
      const response = await fetch(`${API_URL}/api/auth/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        // Provide more specific error messages
        if (response.status === 401) {
          throw new Error('Invalid email or password');
        } else if (response.status === 500) {
          throw new Error('Server error. Please try again later');
        }
        throw new Error(data.details || data.error || 'Login failed');
      }

      // Success
      localStorage.setItem('auth_token', data.token);
      setToken(data.token);
      setUser(data.user);
      return data;
    } catch (error) {
      // Handle network errors
      if (error.message === 'Failed to fetch' || error.name === 'TypeError') {
        throw new Error('Network error. Please check your connection');
      }
      throw error;
    }
  };

  const signup = async (userData) => {
    try {
      const response = await fetch(`${API_URL}/api/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          name: userData.name,
          email: userData.email,
          password: userData.password
        }),
      });

      const data = await response.json();

      if (!response.ok) {
        // Provide more specific error messages
        if (response.status === 409 || data.error?.includes('exists')) {
          throw new Error('Email already exists. Please login or use a different email');
        } else if (response.status === 500) {
          throw new Error('Server error. Please try again later');
        }
        throw new Error(data.details || data.error || 'Signup failed');
      }

      // Success
      localStorage.setItem('auth_token', data.token);
      setToken(data.token);
      setUser(data.user);
      return data;
    } catch (error) {
      // Handle network errors
      if (error.message === 'Failed to fetch' || error.name === 'TypeError') {
        throw new Error('Network error. Please check your connection');
      }
      throw error;
    }
  };

  const completeOnboarding = async (experienceData) => {
    try {
      const response = await fetch(`${API_URL}/api/auth/onboarding`, {
        method: 'POST',
        headers: { 
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`
        },
        body: JSON.stringify(experienceData),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Failed to complete onboarding');
      }

      // Update user state with onboarding data
      setUser(data.user);
      return data;
    } catch (error) {
      if (error.message === 'Failed to fetch' || error.name === 'TypeError') {
        throw new Error('Network error. Please check your connection');
      }
      throw error;
    }
  };

  const logout = () => {
    localStorage.removeItem('auth_token');
    setToken(null);
    setUser(null);
    // Use window.location to ensure proper redirect to the logout page
    // This will handle GitHub Pages subdirectory properly
    window.location.href = '/physical-ai-hackathon/logout';
  };

  return (
    <AuthContext.Provider value={{ user, token, login, signup, logout, completeOnboarding, loading }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => useContext(AuthContext);
