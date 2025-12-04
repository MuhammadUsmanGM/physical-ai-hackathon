import React, { useState, useEffect } from 'react';
import { useAuth } from '../contexts/AuthContext';
import styles from './PersonalizeButton.module.css';

export default function PersonalizeButton() {
  const { user } = useAuth();
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [userLevel, setUserLevel] = useState('intermediate');

  useEffect(() => {
    if (user && user.programming_languages) {
      // Determine overall skill level from user's languages
      const levels = Object.values(user.programming_languages);
      const hasAdvanced = levels.some(l => l === 'advanced' || l === 'expert');
      const hasBeginner = levels.some(l => l === 'beginner');
      
      if (hasAdvanced && !hasBeginner) {
        setUserLevel('advanced');
      } else if (hasBeginner && !hasAdvanced) {
        setUserLevel('beginner');
      } else {
        setUserLevel('intermediate');
      }
    }
  }, [user]);

  const personalizeContent = () => {
    setIsLoading(true);
    
    const contentElement = document.querySelector('article .markdown, article .theme-doc-markdown');
    if (!contentElement) {
      setIsLoading(false);
      return;
    }

    if (!isPersonalized) {
      // Apply personalization
      contentElement.classList.add('personalized');
      contentElement.classList.add(`level-${userLevel}`);
      
      // Add tooltips to technical terms
      addTooltipsToTechnicalTerms(contentElement, userLevel);
      
      // Highlight relevant language examples
      if (user && user.programming_languages) {
        highlightKnownLanguages(contentElement, user.programming_languages);
      }
      
      // Add skill-based hints
      addSkillHints(contentElement, userLevel);
      
      setIsPersonalized(true);
    } else {
      // Remove personalization
      contentElement.classList.remove('personalized');
      contentElement.classList.remove(`level-${userLevel}`);
      
      // Remove all added elements
      contentElement.querySelectorAll('.tooltip-wrapper, .skill-hint, .lang-highlight').forEach(el => {
        if (el.classList.contains('tooltip-wrapper')) {
          const text = el.textContent;
          el.replaceWith(document.createTextNode(text));
        } else {
          el.remove();
        }
      });
      
      setIsPersonalized(false);
    }
    
    setIsLoading(false);
  };

  const addTooltipsToTechnicalTerms = (content, level) => {
    if (level !== 'beginner') return;

    const technicalTerms = {
      'ROS 2': 'Robot Operating System 2 - A framework for robot software development',
      'URDF': 'Unified Robot Description Format - XML format for robot models',
      'Gazebo': 'A 3D robot simulator with physics engine',
      'SLAM': 'Simultaneous Localization and Mapping - Robot navigation technique',
      'Isaac Sim': 'NVIDIA\'s photorealistic robot simulation platform',
      'LiDAR': 'Light Detection and Ranging - Laser-based distance sensor',
      'IMU': 'Inertial Measurement Unit - Sensor for orientation and acceleration',
      'VLA': 'Vision-Language-Action - AI model combining vision, language, and robot actions'
    };

    const walker = document.createTreeWalker(
      content,
      NodeFilter.SHOW_TEXT,
      null,
      false
    );

    const nodesToReplace = [];
    let node;
    
    while (node = walker.nextNode()) {
      const text = node.nodeValue;
      for (const [term, definition] of Object.entries(technicalTerms)) {
        if (text.includes(term) && !node.parentElement.closest('.tooltip-wrapper')) {
          nodesToReplace.push({ node, term, definition });
        }
      }
    }

    nodesToReplace.forEach(({ node, term, definition }) => {
      const text = node.nodeValue;
      const parts = text.split(term);
      const fragment = document.createDocumentFragment();
      
      parts.forEach((part, i) => {
        fragment.appendChild(document.createTextNode(part));
        if (i < parts.length - 1) {
          const wrapper = document.createElement('span');
          wrapper.className = 'tooltip-wrapper';
          wrapper.innerHTML = `${term}<span class="tooltip-text">${definition}</span>`;
          fragment.appendChild(wrapper);
        }
      });
      
      node.parentNode.replaceChild(fragment, node);
    });
  };

  const highlightKnownLanguages = (content, languages) => {
    const knownLangs = Object.keys(languages).filter(lang => 
      languages[lang] === 'advanced' || languages[lang] === 'expert'
    );

    if (knownLangs.length === 0) return;

    // Find code blocks and highlight if they match user's known languages
    content.querySelectorAll('pre code').forEach(codeBlock => {
      const className = codeBlock.className;
      knownLangs.forEach(lang => {
        const langName = lang.toLowerCase();
        if (className.includes(langName) || className.includes(`language-${langName}`)) {
          const highlight = document.createElement('div');
          highlight.className = 'lang-highlight';
          highlight.textContent = `✨ You know ${lang}!`;
          codeBlock.parentElement.insertBefore(highlight, codeBlock);
        }
      });
    });
  };

  const addSkillHints = (content, level) => {
    if (level === 'beginner') {
      // Add "Start Here" hints to important sections
      content.querySelectorAll('h2, h3').forEach((heading, index) => {
        if (index < 2) { // First two headings
          const hint = document.createElement('span');
          hint.className = 'skill-hint beginner-hint';
          hint.textContent = '📚 Start Here';
          heading.appendChild(hint);
        }
      });
    } else if (level === 'advanced') {
      // Add "Advanced" badges to complex sections
      content.querySelectorAll('h2, h3').forEach(heading => {
        const text = heading.textContent.toLowerCase();
        if (text.includes('advanced') || text.includes('optimization') || text.includes('performance')) {
          const hint = document.createElement('span');
          hint.className = 'skill-hint advanced-hint';
          hint.textContent = '🚀 Advanced';
          heading.appendChild(hint);
        }
      });
    }
  };

  if (!user) {
    return null; // Don't show button if not logged in
  }

  return (
    <button
      className={`${styles.personalizeBtn} ${isPersonalized ? styles.active : ''}`}
      onClick={personalizeContent}
      disabled={isLoading}
      title={isPersonalized ? 'Remove Personalization' : 'Personalize Content'}
    >
      {isLoading ? (
        <span className={styles.spinner}>⟳</span>
      ) : (
        <>
          <span className={styles.icon}>✨</span>
          <span className={styles.text}>{isPersonalized ? 'Standard' : 'Personalize'}</span>
        </>
      )}
    </button>
  );
}
