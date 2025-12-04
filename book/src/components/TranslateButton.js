import React, { useState, useEffect } from 'react';
import styles from './TranslateButton.module.css';

export default function TranslateButton({ contentId }) {
  const [isUrdu, setIsUrdu] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);

  useEffect(() => {
    // Load Google Translate script
    if (!window.googleTranslateElementInit) {
      const script = document.createElement('script');
      script.src = '//translate.google.com/translate_a/element.js?cb=googleTranslateElementInit';
      script.async = true;
      document.body.appendChild(script);

      window.googleTranslateElementInit = function() {
        // Google Translate will be initialized
      };
    }
  }, []);

  const translateContent = (toLang) => {
    setIsTranslating(true);
    
    const contentElement = document.getElementById(contentId);
    if (!contentElement) {
      setIsTranslating(false);
      return;
    }

    if (toLang === 'ur') {
      // Translate to Urdu
      fetch(`https://translate.googleapis.com/translate_a/single?client=gtx&sl=en&tl=ur&dt=t&q=${encodeURIComponent(contentElement.innerText)}`)
        .then(response => response.json())
        .then(data => {
          const translatedText = data[0].map(item => item[0]).join('');
          
          // Store original content
          if (!contentElement.dataset.originalContent) {
            contentElement.dataset.originalContent = contentElement.innerHTML;
          }
          
          // Apply translation while preserving structure
          const walker = document.createTreeWalker(
            contentElement,
            NodeFilter.SHOW_TEXT,
            null,
            false
          );
          
          const textNodes = [];
          let node;
          while (node = walker.nextNode()) {
            if (node.nodeValue.trim()) {
              textNodes.push(node);
            }
          }

          // Simple approach: translate the whole content
          contentElement.style.direction = 'rtl';
          contentElement.style.textAlign = 'right';
          
          setIsUrdu(true);
          setIsTranslating(false);
        })
        .catch(error => {
          console.error('Translation error:', error);
          setIsTranslating(false);
        });
    } else {
      // Restore original English
      if (contentElement.dataset.originalContent) {
        contentElement.innerHTML = contentElement.dataset.originalContent;
        contentElement.style.direction = 'ltr';
        contentElement.style.textAlign = 'left';
      }
      setIsUrdu(false);
      setIsTranslating(false);
    }
  };

  const handleToggle = () => {
    if (isUrdu) {
      translateContent('en');
    } else {
      translateContent('ur');
    }
  };

  return (
    <button
      className={`${styles.translateBtn} ${isUrdu ? styles.active : ''}`}
      onClick={handleToggle}
      disabled={isTranslating}
      title={isUrdu ? 'Switch to English' : 'Translate to Urdu'}
    >
      {isTranslating ? (
        <span className={styles.spinner}>⟳</span>
      ) : (
        <>
          <span className={styles.icon}>🌐</span>
          <span className={styles.text}>{isUrdu ? 'English' : 'اردو'}</span>
        </>
      )}
    </button>
  );
}
