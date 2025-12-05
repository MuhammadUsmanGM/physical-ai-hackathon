import React, { useState, useRef, useEffect } from 'react';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './ChatWidget.module.css';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import TextSelectionHandler from './TextSelectionHandler';

export default function ChatWidget() {
  const { user, loading } = useAuth();
  const history = useHistory();
  const [isOpen, setIsOpen] = useState(false);
  const [showLabel, setShowLabel] = useState(true);
  const BotIcon = useBaseUrl('/img/bot.png');
  const [messages, setMessages] = useState([
    {
      role: 'assistant',
      content: 'Hello! I\'m your Physical AI & Humanoid Robotics assistant. Ask me anything about the course!'
    }
  ]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleTextSelected = (text) => {
    setSelectedText(text);
    setIsOpen(true);
    setMessages(prev => [...prev, {
      role: 'system',
      content: `📌 Selected: "${text.substring(0, 150)}${text.length > 150 ? '...' : ''}"`
    }]);
  };

  const handleSend = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage = input.trim();
    const contextText = selectedText;
    setInput('');
    setSelectedText(''); // Clear after sending
    
    setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
    setIsLoading(true);

    try {
      const requestBody = { query: userMessage };
      if (contextText) {
        requestBody.context = contextText;
      }
      
      // const response = await fetch('https://physical-ai-hackathon-production.up.railway.app/chat', {
      // const response = await fetch('http://localhost:8000/chat', { // Local Backend1
      const response = await fetch('https://physical-ai-hackathon-x6me.vercel.app/chat', { // Production Backend1
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      const data = await response.json();
      setMessages(prev => [...prev, { role: 'assistant', content: data.response }]);
    } catch (error) {
      setMessages(prev => [...prev, { 
        role: 'assistant', 
        content: 'Sorry, I encountered an error. Please make sure the backend is running.' 
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  const handleOpenChat = () => {
    setIsOpen(true);
    setShowLabel(false);
  };

  const handleCloseChat = () => {
    setIsOpen(false);
    // Show label again when chat closes (unless user manually closed it)
    if (showLabel !== false) {
      setShowLabel(true);
    }
  };

  // If loading auth state, don't show anything yet
  if (loading) return null;

  return (
    <>
      <TextSelectionHandler onTextSelected={handleTextSelected} />
      
      {/* Floating Chat Button */}
      <button
        className={styles.chatButton}
        onClick={handleOpenChat}
        aria-label="Chat with AI Assistant"
      >
        <img src={BotIcon} alt="AI Assistant" className={styles.botIcon} />
        {showLabel && !isOpen && (
          <span className={styles.buttonLabel}>
            <span className={styles.labelEmoji}>💡</span>
            Ask me about Physical AI!
            <button 
              className={styles.labelClose}
              onClick={(e) => {
                e.stopPropagation();
                setShowLabel(false);
              }}
              aria-label="Hide label"
            >
              ✕
            </button>
          </span>
        )}
      </button>

      {/* Compact Chat Panel */}
      <div className={`${styles.chatPanel} ${isOpen ? styles.open : ''}`}>
        <div className={styles.chatHeader}>
          <div className={styles.headerContent}>
            <img src={BotIcon} alt="AI" className={styles.headerIcon} />
            <div>
              <h3>Physical AI Assistant</h3>
              <p>Your guide to robotics</p>
            </div>
          </div>
          <button 
            className={styles.closeButton}
            onClick={handleCloseChat}
            aria-label="Close chat"
          >
            ✕
          </button>
        </div>

        {!user ? (
          <div className={styles.authPrompt}>
            <div className={styles.authIconWrapper}>
              <span className={styles.authIcon}>🔒</span>
            </div>
            <h4>Unlock AI Assistant</h4>
            <p>Sign in to chat with the Physical AI assistant and get personalized help.</p>
            <button 
              className={styles.loginButton}
              onClick={() => {
                setIsOpen(false);
                history.push('/auth');
              }}
            >
              Login / Sign Up
            </button>
          </div>
        ) : (
          <>
            <div className={styles.chatMessages}>
              {messages.map((msg, idx) => (
                <div 
                  key={idx} 
                  className={`${styles.message} ${styles[msg.role]}`}
                >
                  <div className={styles.messageContent}>
                    {msg.content}
                  </div>
                </div>
              ))}
              {isLoading && (
                <div className={`${styles.message} ${styles.assistant}`}>
                  <div className={styles.messageContent}>
                    <div className={styles.typingIndicator}>
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                  </div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>

            <div className={styles.chatInput}>
              <textarea
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask about ROS 2, Gazebo, Isaac..."
                rows="1"
                disabled={isLoading}
              />
              <button 
                onClick={handleSend}
                disabled={!input.trim() || isLoading}
                className={styles.sendButton}
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none">
                  <path d="M22 2L11 13M22 2L15 22L11 13M22 2L2 9L11 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
              </button>
            </div>
          </>
        )}
      </div>

      {/* Overlay */}
      {isOpen && <div className={styles.overlay} onClick={handleCloseChat} />}
    </>
  );
}
