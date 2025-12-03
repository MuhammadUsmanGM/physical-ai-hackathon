import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg glow-element"
            to="/docs/01-Introduction-to-Physical-AI/">
            Start Your Journey Now 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Course">
      <HomepageHeader />
      <main>
        <section className={styles.featuresSection}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <div className="text--center padding-horiz--md glow-element">
                  <h2>🤖 Explore Physical AI</h2>
                  <p>Dive deep into the revolutionary field of embodied intelligence, where digital minds meet the physical world.</p>
                  <div className="card shadow--tl">
                    <p>Understand how AI systems function in real environments with real physics</p>
                  </div>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md glow-element">
                  <h2>🔧 Master Robotics</h2>
                  <p>Experience hands-on learning with industry-standard platforms like ROS 2, Gazebo, Unity, and NVIDIA Isaac.</p>
                  <div className="card shadow--tl">
                    <p>Build real robots using cutting-edge tools and methodologies</p>
                  </div>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md glow-element">
                  <h2>🚀 Build Intelligent Systems</h2>
                  <p>Design and implement autonomous humanoid robots that interact naturally with humans and environments.</p>
                  <div className="card shadow--tl">
                    <p>Transform concepts into functional, intelligent, embodied systems</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </section>
        
        <section className={`${styles.courseOverview} gradient-bg`}>
          <div className="container padding-vert--xl text--center">
            <h2>🎓 Complete Course Modules</h2>
            <p>Follow our comprehensive learning path to master Physical AI and Humanoid Robotics:</p>
            <div className="row">
              <div className="col">
                <Link to="/docs/01-Introduction-to-Physical-AI/" className="button button--primary button--outline glow-element">
                  <span>🔍</span> Introduction to Physical AI
                </Link>
              </div>
              <div className="col">
                <Link to="/docs/02-The-Robotic-Nervous-System/" className="button button--primary button--outline glow-element">
                  <span>📡</span> ROS 2 Fundamentals
                </Link>
              </div>
              <div className="col">
                <Link to="/docs/03-The-Digital-Twin/" className="button button--primary button--outline glow-element">
                  <span>🎬</span> Simulation (Gazebo & Unity)
                </Link>
              </div>
            </div>
            <div className="row margin-top--lg">
              <div className="col">
                <Link to="/docs/04-The-AI-Robot-Brain/" className="button button--primary button--outline glow-element">
                  <span>🧠</span> NVIDIA Isaac Platform
                </Link>
              </div>
              <div className="col">
                <Link to="/docs/05-Vision-Language-Action/" className="button button--primary button--outline glow-element">
                  <span>💬</span> Vision-Language-Action
                </Link>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}