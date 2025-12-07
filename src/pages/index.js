import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

// Define the content for the three learning points
const learningPoints = [
  {
    title: 'Module 1:  The Robotic Nervous System (ROS 2)',
    description: 'A deep introduction to the communication backbone of modern robots. Students learn how ROS 2 nodes, topics, and services enable distributed control, how Python agents communicate with robotic hardware using rclpy, and how humanoids are described through URDF for motion, balance, and actuation.',
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    description: 'Students build high-fidelity digital twins of humanoid robots for safe testing and experimentation. This module covers physics-accurate simulation in Gazebo, immersive visualization in Unity, and the simulation of critical sensors such as LiDAR, depth cameras, and IMUs.',
  },
  {
    title: 'Module 3:  The AI-Robot Brain (NVIDIA Isaac™)',
    description: 'This module focuses on advanced perception, navigation, and synthetic data generation. Learners use NVIDIA Isaac Sim for photorealistic training environments, Isaac ROS for accelerated VSLAM, and Nav2 for intelligent path planning tailored to bipedal humanoid locomotion.',
  },
   {
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Students explore the future of robotics where language, vision, and control converge. They build voice-activated robot interfaces using Whisper, create cognitive planners that turn natural language into ROS 2 action sequences, and complete a capstone project where a humanoid robot autonomously interprets, plans, and executes commands in a dynamic environment.',
  },
];

// Component for a single learning point card
function LearningPoint({title, description}) {
  return (
    <div className={clsx('col col--4 margin-bottom--lg')}>
      <div className={clsx('card learning-point-card')}>
        <div className="card__header">
          <h3>{title}</h3>
        </div>
        <div className="card__body">
          <p>{description}</p>
        </div>
      </div>
    </div>
  );
}


// Component for the homepage header (Hero section)
function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header
      className="hero hero--primary hero-custom"
      style={{
        padding: '6rem 1rem',
        textAlign: 'center',
        background: 'linear-gradient(135deg, #008080 0%, #20c997 100%)',
        color: '#ffffff',
      }}>
      <div className="container">
        <h1
          className="hero__title"
          style={{
            fontSize: '3.2rem',
            fontWeight: '800',
            marginBottom: '1rem',
            letterSpacing: '-0.5px',
          }}>
          {siteConfig.title}
        </h1>

        <p
          className="hero__subtitle"
          style={{
            fontSize: '1.4rem',
            opacity: 0.9,
            marginBottom: '2rem',
            fontWeight: '400',
          }}>
          {siteConfig.tagline}
        </p>

        <div style={{marginTop: '1.5rem'}}>
          <Link
            className="button button--secondary hero__button"
            style={{
              padding: '0.9rem 2.2rem',
              fontSize: '1.1rem',
              fontWeight: '600',
              borderRadius: '10px',
              backgroundColor: '#ffffff',
              color: '#008080',
              border: 'none',
            }}
            to="/docs/SUMMARY">
            Start Reading
          </Link>
        </div>
      </div>
    </header>
  );
}



// Component for the "What You Will Learn" section
function HomepageLearningPoints() {
  return (
    <section
      className="homepage-learning-points-section"
      style={{
        padding: '5rem 1rem',
        background: 'linear-gradient(180deg, #f0fffa 0%, #e8fff7 100%)',
      }}>
      <div className="container">
        <h2
          className="homepage-learning-points-title"
          style={{
            textAlign: 'center',
            fontSize: '2.5rem',
            fontWeight: '800',
            color: '#006d6d',
            marginBottom: '3rem',
          }}>
          What You Will Learn
        </h2>

        <div className="row" style={{justifyContent: 'center'}}>
          {learningPoints.map((props, idx) => (
            <LearningPoint key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}


// Main Home component
export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="An AI-Native Textbook for a University-Level Capstone Course in Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageLearningPoints />
      </main>
    </Layout>
  );
}
