import React, { useState, useEffect } from 'react';
import styles from './TextSelectionHandler.module.css';

export default function TextSelectionHandler({ onTextSelected }) {
  const [selectedText, setSelectedText] = useState('');
  const [showButton, setShowButton] = useState(false);
  const [buttonPosition, setButtonPosition] = useState({ top: 0, left: 0 });

  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text.length > 0) {
        setSelectedText(text);
        
        // Get selection position
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();
        
        // Position button near selection
        setButtonPosition({
          top: rect.bottom + window.scrollY + 5,
          left: rect.left + window.scrollX + (rect.width / 2)
        });
        
        setShowButton(true);
      } else {
        setShowButton(false);
      }
    };

    const handleMouseDown = (e) => {
      // Hide button if clicking outside
      if (!e.target.closest(`.${styles.selectionButton}`)) {
        setShowButton(false);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('mousedown', handleMouseDown);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('mousedown', handleMouseDown);
    };
  }, []);

  const handleAskAboutText = () => {
    onTextSelected(selectedText);
    setShowButton(false);
    window.getSelection().removeAllRanges(); // Clear selection
  };

  if (!showButton) return null;

  return (
    <button
      className={styles.selectionButton}
      style={{
        position: 'absolute',
        top: `${buttonPosition.top}px`,
        left: `${buttonPosition.left}px`,
        transform: 'translateX(-50%)'
      }}
      onClick={handleAskAboutText}
    >
      <span className={styles.icon}>💬</span>
      <span className={styles.text}>Ask about this</span>
    </button>
  );
}
