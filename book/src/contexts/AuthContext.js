import React, { createContext, useContext, useState, useEffect } from 'react';
import { authClient } from '../lib/auth-client';

const AuthContext = createContext(null);

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [loading, setLoading] = useState(true);
  const [session, setSession] = useState(null);

  useEffect(() => {
    // Check session on mount
    const checkSession = async () => {
      try {
        const { data } = await authClient.getSession();
        if (data) {
          setUser(data.user);
          setSession(data.session);
        }
      } catch (error) {
        console.error('Session check failed', error);
      } finally {
        setLoading(false);
      }
    };
    
    checkSession();
  }, []);

  const login = async (email, password) => {
    const { data, error } = await authClient.signIn.email({
      email,
      password,
    });
    
    if (error) throw new Error(error.message);
    
    if (data) {
      setUser(data.user);
      setSession(data.session);
      return data;
    }
  };

  const signup = async (userData) => {
    const { data, error } = await authClient.signUp.email({
      email: userData.email,
      password: userData.password,
      name: userData.name,
    });

    if (error) throw new Error(error.message);

    if (data) {
      setUser(data.user);
      setSession(data.session);
      return data;
    }
  };

  const logout = async () => {
    await authClient.signOut();
    setUser(null);
    setSession(null);
  };

  return (
    <AuthContext.Provider value={{ user, session, login, signup, logout, loading }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => useContext(AuthContext);
