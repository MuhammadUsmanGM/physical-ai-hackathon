import React, { useState } from 'react';
import DocItem from '@theme-original/DocItem';
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import styles from './styles.module.css';

export default function DocItemWrapper(props) {
  const [isUrdu, setIsUrdu] = useState(false);
  const [isTranslating, setIsTranslating] = useState(false);

  const translateContent = async (toLang) => {
    setIsTranslating(true);
    const contentElement = document.querySelector('article .markdown, article .theme-doc-markdown');
    
    if (!contentElement) {
      setIsTranslating(false);
      return;
    }

    if (toLang === 'ur') {
      // Store original content
      if (!contentElement.dataset.originalHtml) {
        contentElement.dataset.originalHtml = contentElement.innerHTML;
      }

      // Get all text content
      const textContent = contentElement.innerText;

      try {
        // Use Google Translate API (free endpoint)
        const response = await fetch(
          `https://translate.googleapis.com/translate_a/single?client=gtx&sl=en&tl=ur&dt=t&q=${encodeURIComponent(textContent)}`
        );
        const data = await response.json();
        const translatedText = data[0].map(item => item[0]).join('');

        // Apply RTL styling
        contentElement.style.direction = 'rtl';
        contentElement.style.textAlign = 'right';
        contentElement.style.fontFamily = 'Noto Nastaliq Urdu, Arial, sans-serif';

        // Replace text content while preserving structure
        const paragraphs = contentElement.querySelectorAll('p, li, h2, h3, h4, h5, h6, td, th');
        const translatedLines = translatedText.split('\n');
        let lineIndex = 0;

        paragraphs.forEach(p => {
          if (p.textContent.trim() && lineIndex < translatedLines.length) {
            const trimmedLine = translatedLines[lineIndex].trim();
            if (trimmedLine) {
              p.textContent = trimmedLine;
            }
            lineIndex++;
          }
        });

        setIsUrdu(true);
      } catch (error) {
        console.error('Translation error:', error);
        alert('Translation failed. Please try again.');
      }
      setIsTranslating(false);
    } else {
      // Restore original
      if (contentElement.dataset.originalHtml) {
        contentElement.innerHTML = contentElement.dataset.originalHtml;
        contentElement.style.direction = 'ltr';
        contentElement.style.textAlign = 'left';
        contentElement.style.fontFamily = '';
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
    <div className={styles.docItemContainer}>
      <div className={styles.translateButtonWrapper}>
        <PersonalizeButton />
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
      </div>
      <DocItem {...props} />
    </div>
  );
}
