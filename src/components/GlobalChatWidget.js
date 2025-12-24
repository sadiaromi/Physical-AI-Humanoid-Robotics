import React from 'react';
import ChatWidget from './chat_widget';

// Simple wrapper component to host the ChatWidget globally
export default function GlobalChatWidget() {
  return (
    <div id="global-chat-widget">
      <ChatWidget />
    </div>
  );
}